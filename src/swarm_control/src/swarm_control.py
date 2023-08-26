#!/usr/bin/env python3
import rospy

# import State2D.msg
from swarm_msgs.msg import State2D
from geometry_msgs.msg import Twist, PoseStamped
import geometry_msgs.msg

import tf2_ros
import tf2_msgs.msg
from std_msgs.msg import Bool, Int32
import tf_conversions # quaternion stuff

from utilities.safe_swarm_controller import *

import numpy as np

import time

import shapely
import shapely.geometry
import shapely.affinity

from arm_msgs.srv import FleetInformation, FleetInformationRequest,\
        MachineStatus, MachineStatusRequest,\
        RobotAssignments, RobotAssignmentsRequest,\
        TicketList, TicketListRequest

'''
swarm_control.py
Alex Elias, Burak Aksoy

Commands robot states (position and velocity) to achieve rigid-body motion

Parameters:

    desired_swarm_vel_topic_name:
    just_swarm_frame_vel_input_topic_name:

    N_robots: Numer of robots to control

    theta_scale: How much we weigh theta vs x,y in choosing safe swarm velocity
        Higher theta_scale ==> Swarm velocity follows angle commands closer
        (But doesn't follow XY commands as closely)

    Repeat for all N robots:
        just_robot_vel_input_topic_name_0:
        state_publish_topic_name_0: State2D command for robot 0
        tf_frame_name_0: Frame name (for rviz) for robot 0
        vel_lim_x_0: Velocity limit in X for robot 0
        vel_lim_y_0: Velocity limit in Y for robot 0
        vel_lim_theta_0: Velocity limit in theta for robot 0
        acc_lim_x_0: Acceleration limit in X for robot 0
        acc_lim_y_0: Acceleration limit in Y for robot 0
        acc_lim_theta_0: Acceleration limit in theta for robot 0

'''

# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.1
log_tag = "Swarm Controller"

class Swarm_Control:
    def __init__(self):
        rospy.init_node('swarm_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"{log_tag}: Node started.")

        # Team-level parameters. Lists of strings/ints.
        self.team_command_topics = []
        self.team_frame_command_topics = []
        self.team_footprint_topics = []
        self.tf_changer_topics = []
        self.team_sizes = []
        self.robot_ids = []

        # Robot enable status and theta scale.
        robot_enable_status_topic = rospy.get_param('~robot_enable_status_topic')
        self.theta_scale = rospy.get_param('~theta_scale')

        # Parameters for the entire fleet. These do not change with team
        # changes and only need to be populated once.
        # Size of the robot fleet.
        self.fleet_size = 0

        # These are lists of strings that will change based on current teams.
        self.robot_frame_command_topics = []
        self.state_publish_topics = []
        self.virtual_robot_frame_names = []

        # Request information about the fleet.
        self.request_fleet_information()

        # Robot footprints
        self.enabled_robots = []
        self.footprints = [None]*self.fleet_size
        self.v_max = np.zeros((3, self.fleet_size))
        self.a_max = np.zeros((3, self.fleet_size))

        for i in range(self.fleet_size):
            # Each element in the footprints list has an Nx2 numpy array that
            # represents the footprint coordinates in the robot center where N
            # is the number points in the footprint of the robot.
            self.footprints[i] = np.array(
                rospy.get_param('~footprint_' + str(i))
            )
            self.v_max[0, i] = rospy.get_param('~vel_lim_x_' 	 + str(i))
            self.v_max[1, i] = rospy.get_param('~vel_lim_y_' 	 + str(i))
            self.v_max[2, i] = rospy.get_param('~vel_lim_theta_' + str(i))
            self.a_max[0, i] = rospy.get_param('~acc_lim_x_' 	 + str(i))
            self.a_max[1, i] = rospy.get_param('~acc_lim_y_'	 + str(i))
            self.a_max[2, i] = rospy.get_param('~acc_lim_theta_' + str(i))

            self.enabled_robots.append(True)

        # Subscriber for the robot enable status.
        rospy.Subscriber(robot_enable_status_topic, Int32, self.robot_enable_changer, queue_size=5)

        # Subscribers to each robot's frame command topic.
        self.robot_frame_command_subs = [None]*self.fleet_size
        for i in range(self.fleet_size):
            self.robot_frame_command_subs[i] = rospy.Subscriber(
                self.robot_frame_command_topics[i],
                Twist,
                self.just_robot_velocity_callback,
                i,
                queue_size=1
            )

        # Publishers for individual robot desired states.
        self.robot_command_pubs = [None]*self.fleet_size
        for i in range(self.fleet_size):
            self.robot_command_pubs[i] = rospy.Publisher(
                self.state_publish_topics[i], State2D, queue_size=1
            )

        # Initialize local variables to keep track of robot frames.
        self.robots_xyt = np.zeros((3, self.fleet_size))
        self.robots_last_velocities = np.zeros((3, self.fleet_size))
        self.v_robots_prev = np.zeros((3, self.fleet_size))
        self.last_timestep_requests = {}

        self.swarm_xyt = np.zeros((3,1))
        rospy.Subscriber(desired_swarm_vel_topic_name, Twist, self.desired_swarm_velocity_callback, queue_size=1)
        rospy.Subscriber(just_swarm_frame_vel_input_topic_name, Twist, self.just_swarm_frame_velocity_callback, queue_size=1)
        rospy.Subscriber(sync_frame_topic_name, PoseStamped, self.frame_changer_callback, queue_size=20)

        # Swarm footprint polygon publisher
        self.footprint_publish_topic_name = rospy.get_param('~footprint_publish_topic_name', "swarm_footprint")
        self.footprint_pub = rospy.Publisher(self.footprint_publish_topic_name, geometry_msgs.msg.PolygonStamped, queue_size=1)

        # Footprint publish timer
        footprint_update_rate = 3.0
        rospy.Timer(rospy.Duration(1.0 / footprint_update_rate), self.publish_formation_footprint_polygon)


        # TF Broadcasts are sent at 30Hz.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        tf_update_rate = 30.0
        rospy.Timer(rospy.Duration(1.0 / tf_update_rate), self.publish_tf_frames)


    def robot_enable_changer(self,data):
        number=data.data
        rospy.logwarn(str(len(self.enabled_robots)))
        rospy.logwarn(str(self.fleet_size))
        for i in range(self.fleet_size-1,-1,-1):
            if(number>>i==1):
                number-=pow(2,i)
                
                self.enabled_robots[i]=False
                rospy.logwarn("disabled robot: "+str(i+1)+"status: "+str(self.enabled_robots[i]))
                self.robots_last_velocities[:,i] = 0
            else:
                
                self.enabled_robots[i]=True
                rospy.logwarn("enabled robot: "+str(i+1)+"status: "+str(self.enabled_robots[i]))

    def frame_changer_callback(self,data):
        if(data.header.frame_id in self.tf_frame_names):
            array_position=self.tf_frame_names.index(data.header.frame_id)
            orientations= [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
            (roll,pitch,yaw)=tf_conversions.transformations.euler_from_quaternion(orientations)
            rospy.logwarn(np.array2string(self.robots_xyt))
            self.robots_xyt[0,array_position]=data.pose.position.x
            self.robots_xyt[1,array_position]=data.pose.position.y
            self.robots_xyt[2,array_position]=yaw

    def desired_swarm_velocity_callback(self, data):
        if sum(self.enabled_robots) == 0:
            return

        dt = self.get_timestep("desired_swarm_velocity")
        if dt == 0: # Exceeded MAX_TIMESTEP
            self.v_robots_prev *= 0.
            return

        v_desired = np.zeros((3,1))
        v_desired[0, 0] = data.linear.x
        v_desired[1, 0] = data.linear.y
        v_desired[2, 0] = data.angular.z


        enabled_index = np.where(self.enabled_robots)[0]

        p_i_mat = self.robots_xyt[0:1+1,enabled_index]
        theta_vec = self.robots_xyt[[2],[enabled_index]]
        
        v_i_world, v_i_rob, xyt_i, v, xyt_swarm_next = safe_motion_controller(
            v_desired, self.theta_scale, p_i_mat, theta_vec,
            self.v_max[:, enabled_index], self.a_max[:, enabled_index], dt, sum(self.enabled_robots),
            self.v_robots_prev[:, enabled_index], self.swarm_xyt)

        # Don't update self.robots_xyt, since that's in the swarm frame
        self.v_robots_prev[:, enabled_index] = v_i_rob;
        self.swarm_xyt = xyt_swarm_next;

        # Send desired state to each robot
        for i in range(sum(self.enabled_robots)):
            i_total = enabled_index[i]
            
            s = State2D();
            s.pose.x = xyt_i[0,i]
            s.pose.y = xyt_i[1,i]
            s.pose.theta = xyt_i[2,i]
            s.twist.linear.x = v_i_world[0,i]
            s.twist.linear.y = v_i_world[1,i]
            s.twist.angular.z = v_i_world[2,i]
            self.robot_command_pubs[i_total].publish(s)

    def just_swarm_frame_velocity_callback(self, data):
        '''
        Move the position of the swarm frame without moving the robots
        i.e. move the swarm frame, and move the robots
            w.r.t. the swarm frame in the opposite direction
        '''
        dt = self.get_timestep("desired_swarm_frame_velocity")

        qd_world = np.zeros((3,1))
        qd_world[0, 0] = data.linear.x
        qd_world[1, 0] = data.linear.y
        qd_world[2, 0] = data.angular.z

        qd_swarm = rot_mat_3d(-self.swarm_xyt[2, 0]).dot(qd_world)

        q_world_delta = dt * qd_world
        q_swarm_delta = dt * qd_swarm

        self.swarm_xyt = self.swarm_xyt + q_world_delta

        #  np.diag([1., 1., 0.]).
        self.robots_xyt = rot_mat_3d(-q_swarm_delta[2,0]).dot(self.robots_xyt  - q_swarm_delta )

    def just_robot_velocity_callback(self, data, i_robot):
        if(self.enabled_robots[i_robot]):
            # Move the position of robot i_robot in the world frame
            dt = self.get_timestep("just_robot_velocty_"+str(i_robot))

            qd_world = np.zeros((3,1))
            qd_world[0, 0] = data.linear.x
            qd_world[1, 0] = data.linear.y
            qd_world[2, 0] = data.angular.z

            # robots_xyt is in the swarm frame
            qd_swarm = rot_mat_3d(-self.swarm_xyt[2]).dot(qd_world)
            self.robots_xyt[:,i_robot] = self.robots_xyt[:,i_robot] + dt * qd_swarm.flatten()
            #print(self.robots_xyt)


    def get_timestep(self, integrator_name):
        current_time = time.time()
        if integrator_name in self.last_timestep_requests:
            dt = current_time - self.last_timestep_requests[integrator_name]
            self.last_timestep_requests[integrator_name] = current_time
            if dt > MAX_TIMESTEP:
                dt = 0.0
            return dt
        else:
            self.last_timestep_requests[integrator_name] = current_time
            return 0.0

    
    def publish_tf_frames(self,event):
        tf_swarm_frame = xyt2TF(self.swarm_xyt, "map", "swarm_frame")
        self.tf_broadcaster.sendTransform(tf_swarm_frame)

        for i in range(self.fleet_size):
            if(self.enabled_robots[i]):
                tf_robot_i = xyt2TF(self.robots_xyt[:,i], "swarm_frame", self.tf_frame_names[i])
                self.tf_broadcaster.sendTransform(tf_robot_i)


    def publish_formation_footprint_polygon(self,event):
        if sum(self.enabled_robots) == 0:
            return
        
        # Find the points in the active formation
        coords = []
        coords.append([0.0,0.0]) # Include the location of the swarm frame to the footprint
        default_footprint = [[0.345, -0.260], [0.345, 0.260], [-0.345, 0.260], [-0.345, -0.260]]
        for point in default_footprint:
            coords.append(point)

        for i in range(self.fleet_size):
            if(self.enabled_robots[i]):
                xyt = self.robots_xyt[:,i].flatten() # in swarm frame pose

                robot_pts = self.footprints[i] # Nx2
                # convert to shapely multi point object
                robot_pts_multi_pt = shapely.geometry.MultiPoint(robot_pts)
                # rotate
                robot_pts_multi_pt = shapely.affinity.rotate(robot_pts_multi_pt, xyt[2], origin=(0,0), use_radians=True)
                # translate
                robot_pts_multi_pt = shapely.affinity.translate(robot_pts_multi_pt, xoff=xyt[0], yoff=xyt[1])

                for point in robot_pts_multi_pt.geoms:
                    coords.append(list(point.coords[0]))

        # Find the convex hull of the points with shapely
        multi_pt = shapely.geometry.MultiPoint(coords)
        hull = multi_pt.convex_hull
        
        # NOTE: convex_hull will return a polygon if your hull contains more than two points. If it only contains two points, you'll get a LineString, and if it only contains one point, you'll get a Point.

        # Convert the hull points to ROS Point32 messages
        polygon = []
        if hull.geom_type == 'Polygon':
            for pt in list(hull.exterior.coords):
                point32 = geometry_msgs.msg.Point32()
                point32.x = pt[0]
                point32.y = pt[1]
                point32.z = 0  # assuming 2D coordinates
                polygon.append(point32)
        else:
            rospy.loginfo("Not enough points to form a polygon.")
            return
        
        # Prepare the msg and publish
        msg = geometry_msgs.msg.PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "swarm_frame"  # replace with appropriate frame
        msg.polygon.points = polygon
        self.footprint_pub.publish(msg)

    def shutdown(self):
        '''.'''
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def request_ticket_list(self):
        '''Request the current ticket list from the ticket_service.'''
        rospy.wait_for_service('ticket_service')
        try:
            # TicketListRequest() is empty.
            request = TicketListRequest()
            ticket_list = rospy.ServiceProxy('ticket_service', TicketList)
            response = ticket_list(request)

            # self.all_tickets = convert_ticket_list_to_task_dict(response.all_tickets)
            # self.waiting = response.waiting
            # self.ready = response.ready
            self.ongoing = response.ongoing
            # self.done = response.done
        except rospy.ServiceException as e:
            rospy.logerr(f'{log_tag}: Ticket list request failed: {e}.')

    def request_assigned_robot_information(self, ticket_id: int):
        '''Request the .'''
        rospy.wait_for_service('robot_assignments_service')
        try:
            request = RobotAssignmentsRequest()
            request.ticket_id = ticket_id
            robot_assignments = rospy.ServiceProxy(
                'robot_assignments_service',
                RobotAssignments
            )
            response = robot_assignments(request)

            self.num_robots = response.num_assigned_robots
            self.robot_ids = response.robot_ids
            # self.robot_names = response.robot_names
            # self.robot_frame_command_topics = response.robot_frame_command_topics
            # self.robot_command_topics = response.robot_command_topics
            # self.virtual_robot_frame_names = response.virtual_robot_frame_names
            # self.real_robot_frame_names = response.real_robot_frame_names

            self.team_id = response.team_id

            self.team_command_topic = response.team_command_topic
            self.team_frame_command_topic = response.team_frame_command_topic
            self.team_footprint_topic = response.team_footprint_topic
            self.team_tf_frame = response.team_tf_frame
            self.tf_changer_topic = response.tf_changer_topic

            if self.tf_changer is not None:
                self.tf_changer.unregister()

            if len(self.tf_changer_topic) > 0:
                self.tf_changer = rospy.Publisher(
                    self.tf_changer_topic, PoseStamped, queue_size=10
                )

            self.node_names = []
            for robot in range(self.num_robots):
                self.node_names.append(response.robot_node_names[robot].string_list)
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Robot assignment request failed: {e}.")

    def request_fleet_information(self):
        '''Requests the information about all robots in the fleet.

        The information includes the total number of robots in the fleet,
        robot names, topics, and frame names.
        '''
        rospy.wait_for_service('fleet_information_service')
        try:
            request = FleetInformationRequest()
            fleet_information = rospy.ServiceProxy(
                "fleet_information_service",
                FleetInformation
            )
            response = fleet_information(request)

            self.robot_frame_command_topics = response.robot_frame_command_topics
            self.state_publish_topics = response.robot_desired_state_topics
            self.virtual_robot_frame_names = response.virtual_robot_frame_names
            self.fleet_size = response.fleet_size
        except:
            pass

            
def rot_mat_3d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    
def wrapToPi(a):
    '''
    Wraps angle to [-pi,pi)
    '''
    return ((a+np.pi) % (2*np.pi))-np.pi

def xyt2TF(xyt, header_frame_id, child_frame_id):
    '''
    Converts a numpy vector [x; y; theta]
    into a tf2_msgs.msg.TFMessage message
    '''
    xyt = xyt.flatten()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = header_frame_id
    #t.header.stamp = ros_time #rospy.Time.now()
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = xyt[0]
    t.transform.translation.y = xyt[1]
    t.transform.translation.z = 0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyt[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t

if __name__ == '__main__':
    swarm_control = Swarm_Control()
    rospy.spin()
