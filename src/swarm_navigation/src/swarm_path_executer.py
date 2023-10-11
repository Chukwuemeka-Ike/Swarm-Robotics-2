#!/usr/bin/env python3

import pandas as pd
import rospy
import numpy as np

import threading
import time
import os

from swarm_msgs.msg import State2D
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

import arm_msgs.msg # TicketMotionParams

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

import tf_conversions # quaternion stuff

from costmap_parameter_updater import * # CostmapParameterUpdater class



class PathExecuter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('swarm_path_executer', anonymous=False)

        self.costmap_updater_planner_name = rospy.get_param("~costmap_updater_planner_name", "global_planner")
        self.costmap_updater_costmap_name = rospy.get_param("~costmap_updater_costmap_name", "costmap")
        self.costmap_updater_inflation_layer_name = rospy.get_param("~costmap_updater_inflation_layer_name", "inflater_layer")
        self.costmap_updater_service_wait_timeout = rospy.get_param("~costmap_updater_service_wait_timeout", 2)
        self.costmap_updater_num_trials = rospy.get_param("~costmap_updater_num_trials", 5)
        self.costmap_updater_trial_intervals = rospy.get_param("~costmap_updater_trial_intervals", 0.5)

        # Create costmap_parameter_updater object
        self.costmap_updater = CostmapParameterUpdater(self.costmap_updater_planner_name,
                                                       self.costmap_updater_costmap_name,
                                                       self.costmap_updater_inflation_layer_name,
                                                       self.costmap_updater_service_wait_timeout,
                                                       self.costmap_updater_num_trials,
                                                       self.costmap_updater_trial_intervals)
        
        self.footprint_topic_name = rospy.get_param("~footprint_topic_name", "swarm_footprint") # subscribed

        self.plan_request_goal_topic_name = rospy.get_param("~plan_request_goal_topic_name", "/global_planner/goal") # published
        self.plan_response_path_topic_name = rospy.get_param("~plan_response_path_topic_name", "/global_planner/planner/plan") # subscribed

        self.simple_goal_topic_name = rospy.get_param("~simple_goal_topic_name", "/move_base_simple/goal") # subscribed

        self.saved_path_files_directory = rospy.get_param("~saved_path_files_directory", "./") # stores the csv files of the preplanned paths
        self.saved_path_file_topic_name = rospy.get_param("~saved_path_file_topic_name", "path_csv_filename") # subscribed

        self.pub_rate_desired_state = rospy.get_param("~pub_rate_desired_state", 100.0)
        self.waypoint_update_rate = rospy.get_param("~waypoint_update_rate", 50.0)

        self.position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name', "main/swarm_frame/odom") # subscribed

        self.waypoint_dist_tolerance = rospy.get_param("~waypoint_dist_tolerance", 0.05) # meters
        self.waypoint_ori_tolerance  = np.deg2rad(rospy.get_param("~waypoint_ori_tolerance", 10)) # rad

        self.planner_trigger_dist_threshold = rospy.get_param("~planner_trigger_dist_threshold", 0.15) # meters

        self.wait_for_plan_timeout = rospy.get_param("~wait_for_plan_timeout", 3.0) # seconds
        self.new_plan_recentness_threshold = rospy.get_param("~new_plan_recentness_threshold", 1.0) # seconds

        self.desired_state_topic_name = rospy.get_param("~desired_state_topic_name", "/swarm/desired_state") # published

        self.execution_disabled = rospy.get_param("~execution_disabled", False) 

        self.update_costmaps_on_every_footprint = rospy.get_param("~update_costmaps_on_every_footprint", False) 

        # Service names
        self.disable_execution_service_name = rospy.get_param("~disable_execution_service_name", "disable_path_execution")
        self.enable_execution_service_name = rospy.get_param("~enable_execution_service_name", "enable_path_execution")
        self.cancel_execution_service_name = rospy.get_param("~cancel_execution_service_name", "cancel_path_execution")
        self.toggle_adjust_path_service_name = rospy.get_param("~toggle_adjust_path_service_name", "toggle_adjust_path") 

        # Service to disable the path executions
        self.srv_disable_execution = rospy.Service(self.disable_execution_service_name, 
                                                    Trigger, 
                                                    self.srv_disable_execution_cb)
        
        # Service to enable the path executions
        self.srv_enable_execution = rospy.Service(self.enable_execution_service_name, 
                                                    Trigger, 
                                                    self.srv_enable_execution_cb)

        # Service to cancel the path executions
        self.srv_cancel_execution = rospy.Service(self.cancel_execution_service_name, 
                                                    Trigger, 
                                                    self.srv_cancel_execution_cb)

        # Service to toggle the manual path adjustment (enable/disable)
        self.adjust_path_enabled = False # by default manual path adjusting is disabled
        self.execution_disabled_last_state = None
        self.adjusted_pos_start = np.zeros(2)
        self.adjusted_ori_start = 0.0
        self.adjusted_pos = np.zeros(2)
        self.adjusted_ori = 0.0
        self.srv_toggle_adjust_path = rospy.Service(self.toggle_adjust_path_service_name, 
                                                    SetBool, 
                                                    self.srv_toggle_adjust_path_cb)

        
        # Create a publisher for the State2D messages
        self.pub_desired_state = rospy.Publisher(self.desired_state_topic_name, State2D, queue_size=1)


        # Subscribe to Current swarm position
        self.curr_pos = None
        self.curr_ori = None
        rospy.Subscriber(self.position_feedback_topic_name, nav_msgs.msg.Odometry, self.position_feedback_cb, queue_size=1)

        # Subscribe to Simple Goal Pose
        rospy.Subscriber(self.simple_goal_topic_name, geometry_msgs.msg.PoseStamped, self.simple_goal_cb, queue_size=1)

        # Subscribe to current swarm footprint
        self.footprint_lock = threading.Lock()
        self.footprint = []
        rospy.Subscriber(self.footprint_topic_name, geometry_msgs.msg.PolygonStamped, self.footprint_cb, queue_size=1)

        # Create a publisher for the planner to take
        self.pub_plan_goal = rospy.Publisher(self.plan_request_goal_topic_name, geometry_msgs.msg.PoseStamped, queue_size=1)

        # Subscribe to the planner calculated plans
        self.planner_plan_lock = threading.Lock()
        self.planner_plan_csv_reader_lock = threading.Lock()

        self.plan_execute_permit = True

        self.planner_plan = []
        self.planner_plan_last_update_time = 0.0
        rospy.Subscriber(self.plan_response_path_topic_name, nav_msgs.msg.Path, self.planner_response_cb, queue_size=1)

        # Subscribe to the saved path csv file os path string topic
        rospy.Subscriber(self.saved_path_file_topic_name, arm_msgs.msg.TicketMotionParams, self.saved_path_file_cb, queue_size=1)

        # Setup timer for publishing the desired states
        self.waypoint_reached = True
        self.current_waypoint = []
        rospy.Timer(rospy.Duration(1.0 / self.pub_rate_desired_state), self.pub_desired_state_cb)

        # Setup timer for waypoint updating
        rospy.Timer(rospy.Duration(1.0 / self.waypoint_update_rate), self.update_waypoint_cb)

        


    def pub_desired_state_cb(self, event):
        if not self.execution_disabled:
            if self.current_waypoint:
                x = self.current_waypoint[0]
                y = self.current_waypoint[1]
                th = self.current_waypoint[2]

                state2d = State2D()
                state2d.pose = geometry_msgs.msg.Pose2D(x,y,th)
                self.pub_desired_state.publish(state2d)


    def update_waypoint_cb(self, event):
        if not self.execution_disabled:
            if self.curr_pos is None or self.curr_ori is None:
                rospy.logwarn("Current position is not yet set..")
                return
            
            if self.waypoint_reached:
                # Debug info 
                # if len(self.planner_plan) > 0:
                #     rospy.loginfo(f'Current Plan length left: {len(self.planner_plan)}')

                if self.planner_plan: # check if the planner path is empty, if not, get a new waypoint
                    # with self.planner_plan_csv_reader_lock:
                    # with self.planner_plan_lock:
                    if self.plan_execute_permit:
                        self.current_waypoint = self.planner_plan.pop()

                        # Update the waypoint with the manual path adjustments
                        self.current_waypoint[0] = self.current_waypoint[0] + self.adjusted_pos[0] # x
                        self.current_waypoint[1] = self.current_waypoint[1] + self.adjusted_pos[1] # y
                        self.current_waypoint[2] = self.current_waypoint[2] + self.adjusted_ori # th

                        rospy.loginfo(f'Current Plan length left: {len(self.planner_plan)}')
                        self.waypoint_reached = False
                else: # if empty, no waypoint left to publish
                    self.current_waypoint = []
                    self.reset_path_adjustment()
            else:
                if self.current_waypoint:
                    # keep checking whether the waypoint is reached
                    x = self.current_waypoint[0]
                    y = self.current_waypoint[1]
                    th = self.current_waypoint[2]

                    wayp_pos = np.array([x,y])
                    wayp_ori = th

                    pose_dist = np.linalg.norm(self.curr_pos - wayp_pos)
                    pose_ori = abs(self.curr_ori - wayp_ori)

                    if pose_dist <= self.waypoint_dist_tolerance and pose_ori <= self.waypoint_ori_tolerance:
                        self.waypoint_reached = True
            
    def saved_path_file_cb(self,msg):
        rospy.loginfo("Saved Path File Callback is called. ")
        if self.adjust_path_enabled:
            rospy.logwarn("Adjusting waypoint is enabled, Saved Path File Callback is ignored.")
            return

        if self.curr_pos is None or self.curr_ori is None:
            rospy.logwarn("Current position is not yet set, the csv file path is ignored.")
            return
        
        # msg.path_csv_filename arrives as: f"ticket_{self.ticket_id}.csv"
        # Combine the csv file with the path directory
        csv_file_os_path = os.path.join(self.saved_path_files_directory, msg.path_csv_filename)

        # Check if the file exists
        if not os.path.exists(csv_file_os_path):
            rospy.logwarn(f"CSV file {csv_file_os_path} does not exist!")
            return

        
        self.plan_execute_permit = False 
        self.reset_path_adjustment()

        # Read the waypoints from the csv file
        csv_waypoints = []
        try:
            df = pd.read_csv(csv_file_os_path)
            if {"x", "y", "theta"}.issubset(df.columns):
                csv_waypoints = df[["x", "y", "theta"]].values.tolist()
                csv_waypoints.reverse()
                # rospy.loginfo(f'Loaded csv_waypoints: {csv_waypoints}')
                rospy.loginfo(f'Loaded csv_waypoints length: {len(csv_waypoints)}')
            else:
                rospy.logerr(f'CSV file does not contain required columns ("x", "y", "theta")')
        except FileNotFoundError:
            rospy.logerr(f'Could not find the specified CSV file: {csv_file_os_path}')
        except Exception as e:
            rospy.logerr(f'An error occurred while reading CSV file: {e}')

        # the csv points are planned wrt to the needle frame of the machine
        # Therefore we need to convert them to the world frame before executing (TODO)
        
        # Needle location arrives as: msg.needle_location = float32[] [x,y,theta] from machine_params.yaml
        # Machine location arrives as: msg.machine_location = float32[] [x,y,theta] from machine_params.yaml
        # Recall:
        # self.curr_pos = np.array([ data.pose.pose.position.x, data.pose.pose.position.y])
        # self.curr_ori = yaw # theta

        for i,wp in enumerate(csv_waypoints):
            P = np.array(wp[:2]) # vector from needle to waypoint in needle frame
            
            P_wn = np.array(msg.needle_location[:2]) # vector from world to needle in world frame

            P = P_wn + rot_mat(msg.needle_location[2]).dot(P) # vector from world to waypoint in world frame
            csv_waypoints[i] = [P[0],P[1], wrapToPi(wp[2]+msg.needle_location[2])] # [x,y,theta] in world frame

        if csv_waypoints: 
            # Check if the initial waypoint is far away and requires the planner to calculate path to go there
            initial_goal = csv_waypoints.pop()
            goal_ori = initial_goal[2]
            goal_pos = np.array([initial_goal[0],initial_goal[1]])
            
            if np.linalg.norm(self.curr_pos-goal_pos) > self.planner_trigger_dist_threshold:
                
                # Calculate a path from the current pose to the goal with the planner
                msg = geometry_msgs.msg.PoseStamped()

                # Set the header (with current time and frame_id)
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "map"

                msg.pose.position.x = goal_pos[0]
                msg.pose.position.y = goal_pos[1]
                msg.pose.position.z = 0.0  # in 2D

                quat = tf_conversions.transformations.quaternion_from_euler(0, 0, goal_ori) 
                msg.pose.orientation.x = quat[0]
                msg.pose.orientation.y = quat[1]
                msg.pose.orientation.z = quat[2]
                msg.pose.orientation.w = quat[3]

                triggered = self.trigger_path_planner(msg)
                if triggered:
                    # Wait for a new path to be planned
                    plan_updated = self.wait_for_plan_to_update()

                    if plan_updated:
                        # self.planner_plan is updated
                        rospy.loginfo(f'Plan length before appending: {len(self.planner_plan)}')

                        # Append the planner calculated path to the csv read path 
                        self.planner_plan = csv_waypoints + self.planner_plan
                        rospy.loginfo(f'Plan length after appending: {len(self.planner_plan)}')

                        # the rest is handled with the self.pub_desired_state_cb function
                        pass
                    else:
                        rospy.logerr("The planner could not find a path in given" + str(self.wait_for_plan_timeout) + "seconds.")    
                else:
                    rospy.logerr("The goal pose is ignored.")
            else:
                # The csv waypoints are directly represents the plan
                self.planner_plan = csv_waypoints
        else:
            rospy.logwarn("After reading the CSV file, no waypoint is found.")    

        self.plan_execute_permit = True


        
    def simple_goal_cb(self, msg):
        rospy.loginfo("Simple Goal Callback is called. ")

        if self.adjust_path_enabled:
            rospy.logwarn("Adjusting waypoint is enabled, Saved Path File Callback is ignored.")
            return

        if self.curr_pos is None or self.curr_ori is None:
            rospy.logwarn("Current position is not yet set, the goal is ignored.")
            return

        self.plan_execute_permit = False 
        self.reset_path_adjustment()

        orientations = [msg.pose.orientation.x, 
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w]
        (roll,pitch,yaw) = tf_conversions.transformations.euler_from_quaternion(orientations)

        goal_ori = yaw
        goal_pos = np.array([msg.pose.position.x,msg.pose.position.y])

        if np.linalg.norm(self.curr_pos-goal_pos) > self.planner_trigger_dist_threshold:
            # Calculate a path from the current pose to the goal with the planner
            triggered = self.trigger_path_planner(msg)
            if triggered:
                # Wait for a new path to be planned
                plan_updated = self.wait_for_plan_to_update()

                if plan_updated:
                    # self.planner_plan is updated, the rest is handled with the self.pub_desired_state_cb function
                    pass
                else:
                    rospy.logerr("The planner could not find a path in given " + str(self.wait_for_plan_timeout) + " seconds.")    
            else:
                rospy.logerr("The goal pose is ignored.")

        else:
            # Directly send the goal pose to the controller
            # with self.planner_plan_lock:
            self.planner_plan = [[goal_pos[0], goal_pos[1], goal_ori]]
        self.plan_execute_permit = True
            

    def position_feedback_cb(self, data):
        orientations = [data.pose.pose.orientation.x, 
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w]
        (roll,pitch,yaw) = tf_conversions.transformations.euler_from_quaternion(orientations)
        
        self.curr_pos = np.array([ data.pose.pose.position.x, data.pose.pose.position.y])
        self.curr_ori = yaw # theta

    def footprint_cb(self, msg):
        with self.footprint_lock:
            prev_footprint = self.footprint
            self.footprint = []
            # For each point in the polygon message, store the x and y coordinates in the list.
            for point in msg.polygon.points:
                self.footprint.append([point.x, point.y])

        # optionally, update the costmaps everytime a new footprint message arrives:
        if self.update_costmaps_on_every_footprint:
            # Convert inner lists to tuples
            list1 = [tuple(sublist) for sublist in prev_footprint]
            list2 = [tuple(sublist) for sublist in self.footprint]
            if set(list1) != set(list2): # check if the new footprint is different than the previous footprint contents
                self.update_costmaps()

    def update_costmaps(self):
        with self.footprint_lock:
            if not self.footprint: # check if the footprint is empty
                rospy.logerr("The footprint is empty, the costmaps could not be updated.")
                return  False

            success = self.costmap_updater.update_costmap_parameters(self.footprint)
            if not success:
                rospy.logerr("The costmap could not be updated after trials.")
                return False
            
        return True
    
    def trigger_path_planner(self, msg):
        # Update the costmaps with current footprint
        costmaps_updated = self.update_costmaps()

        if costmaps_updated:
            # Trigger the path planner by forwarding the goal message to the planner's goal topic
            self.pub_plan_goal.publish(msg)
            return True
        else:
            rospy.logerr("The path planner could not be triggered.")
            return False

    def wait_for_plan_to_update(self):
        wait_start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() < (wait_start_time + self.wait_for_plan_timeout):
            if (rospy.Time.now().to_sec() - self.planner_plan_last_update_time) < self.new_plan_recentness_threshold:
                return True
            time.sleep(0.1)

        return False
    
    def planner_response_cb(self, msg):
        # with self.planner_plan_lock:
        self.planner_plan = []

        for pose in reversed(msg.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            # z = pose.pose.position.z

            orientations = [pose.pose.orientation.x, 
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w]
            
            (roll,pitch,yaw) = tf_conversions.transformations.euler_from_quaternion(orientations)

            self.planner_plan.append([x,y,yaw])

        self.planner_plan_last_update_time = rospy.Time.now().to_sec()

    def srv_disable_execution_cb(self, req):
        assert isinstance(req, TriggerRequest)
        rospy.loginfo("Disabling the path execution if possible")

        if self.adjust_path_enabled:
            rospy.logwarn("Adjusting waypoint is enabled, Request to disable path execution is ignored.")
            return TriggerResponse(success=False, message="The path execution could not be disabled due to Adjusting waypoint is enabled!")
        
        self.execution_disabled = True
        return TriggerResponse(success=True, message="The path execution is disabled!")
    
    def srv_enable_execution_cb(self, req):
        assert isinstance(req, TriggerRequest)
        rospy.loginfo("Enabling the path execution if possible")

        if self.adjust_path_enabled:
            rospy.logwarn("Adjusting waypoint is enabled, Request to enable path execution is ignored.")
            return TriggerResponse(success=False, message="The path execution could not be enabled due to Adjusting waypoint is enabled!")
        
        self.execution_disabled = False
        return TriggerResponse(success=True, message="The path execution is enabled!")

    def srv_cancel_execution_cb(self, req):
        assert isinstance(req, TriggerRequest)
        rospy.loginfo("Canceling the path execution if possible")

        if self.adjust_path_enabled:
            rospy.logwarn("Adjusting waypoint is enabled, Request to cancel path execution is ignored.")
            return TriggerResponse(success=False, message="The path execution could not be cancelled due to Adjusting waypoint is enabled!")

        self.plan_execute_permit = False
        self.reset_path_adjustment()
        self.planner_plan = []
        self.plan_execute_permit = True

        return TriggerResponse(success=True, message="The path execution is canceled!")

    def srv_toggle_adjust_path_cb(self,req):
        assert isinstance(req, SetBoolRequest)
        rospy.loginfo("Toggling Adjust Path functionality if possible")
        if self.curr_pos is None or self.curr_ori is None:
            rospy.logwarn("Current position is not yet set..")
            SetBoolResponse(False, "The manual path adjusting could not be toggled, because current swarm position is not yet set")
            return

        if req.data:
            # Store the last state of execution
            self.execution_disabled_last_state = self.execution_disabled
            # Disable the path execution for the safe adjustments
            self.execution_disabled = True

            rospy.loginfo("Attempt to ENABLE Adjust Path")
            self.adjust_path_enabled = True

            # Get the current pose of the swarm
            self.adjusted_pos_start = self.curr_pos
            self.adjusted_ori_start = self.curr_ori

        else:
            rospy.loginfo("Attempt to DISABLE Adjust Path")
            self.adjust_path_enabled = False
            
            # Set the adjustment with the updated pose
            self.adjusted_pos += self.curr_pos - self.adjusted_pos_start
            self.adjusted_ori += self.curr_ori - self.adjusted_ori_start

            # # Update the active waypoint with the manual path adjustments
            # if self.current_waypoint:
            #     self.current_waypoint[0] = self.current_waypoint[0] + self.adjusted_pos[0] # x
            #     self.current_waypoint[1] = self.current_waypoint[1] + self.adjusted_pos[1] # y
            #     self.current_waypoint[2] = self.current_waypoint[2] + self.adjusted_ori # th

            # Re-enable the path execution if the last state was not disabled
            if not self.execution_disabled_last_state:
                self.execution_disabled = False

        rospy.loginfo("The manual path adjusting is now set to: {}".format(self.adjust_path_enabled))
        return SetBoolResponse(True, "The manual path adjusting is now set to: {}".format(self.adjust_path_enabled))
    
    def reset_path_adjustment(self):
        # This function needs to be called everytime when there is a new goal or path request
        self.adjust_path_enabled = False # by default manual path adjusting is disabled
        self.adjusted_pos_start = np.zeros(2)
        self.adjusted_ori_start = 0.0
        self.adjusted_pos = np.zeros(2)
        self.adjusted_ori = 0.0

    
def rot_mat(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def wrapToPi(a):
    '''
    Wraps angle to [-pi,pi)
    '''
    return ((a+np.pi) % (2*np.pi))-np.pi

if __name__ == '__main__':
    node = PathExecuter()
    rospy.spin()






