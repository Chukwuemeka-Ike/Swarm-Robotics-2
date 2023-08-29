#!/usr/bin/env python3
'''
swarm_control.py
Alex Elias, Burak Aksoy, Chukwuemeka Osaretin Ike

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
import rospy

# import State2D.msg
from swarm_msgs.msg import State2D
from geometry_msgs.msg import Twist, PolygonStamped, PoseStamped
from typing import Tuple
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
        RobotAssignments, RobotAssignmentsRequest,\
        TicketList, TicketListRequest


# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.1
log_tag = "Swarm Controller"


class Swarm_Control:
    def __init__(self):
        rospy.init_node('swarm_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"{log_tag}: Node started.")

        # Parameters for the entire fleet. These do not change with team
        # changes and only need to be requested once from Robot Assigner.
        # Size of the robot fleet.
        self.fleet_size = 0

        # Lists of the topics for all robots. Can be indexed as needed.
        self.robot_frame_command_topics = []
        self.state_publish_topics = []
        self.virtual_robot_frame_names = []

        # Robot enable status and tf changer topics.
        self.tf_changer_topic = ""
        self.robot_enable_status_topic = ""

        # Request fleet information to populate the parameters above.
        self.request_fleet_information()

        # Parameters from main_swarm_control.yaml.
        # Theta scale.
        self.theta_scale = rospy.get_param('~theta_scale')

        # Enabled robots.
        self.enabled_robots = []

        # Robot footprints.
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
        rospy.Subscriber(
            self.robot_enable_status_topic, Int32,
            self.robot_enable_changer, queue_size=5
        )
        rospy.Subscriber(
            self.tf_changer_topic, PoseStamped,
            self.tf_changer_callback, queue_size=20
        )

        # Subscribers to each robot's frame command topic.
        self.robot_frame_command_subs = [None]*self.fleet_size
        for i in range(self.fleet_size):
            self.robot_frame_command_subs[i] = rospy.Subscriber(
                self.robot_frame_command_topics[i],
                Twist,
                self.robot_frame_command_callback,
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

        # Team-level parameters. Dictionaries of strings/ints.
        # These change as tickets are started and ended.
        self.team_command_topics = {}
        self.team_frame_command_topics = {}
        self.team_footprint_topics = {}
        self.team_sizes = {}
        self.assigned_robot_ids = {}
        self.assigned_robot_indices = {}
        self.team_tf_frame_names = {}

        # Dictionary holding team poses. IDs are the team IDs gotten from
        # robot assigner. Each
        self.team_poses = {}
        self.team_command_subs = {}
        self.team_frame_command_subs = {}
        self.team_footprint_pubs = {}
        self.team_footprint_pub_timers = {}


        self.swarm_xyt = np.zeros((3,1))
        rospy.Subscriber(desired_swarm_vel_topic_name, Twist, self.team_command_callback, queue_size=1)
        rospy.Subscriber(just_swarm_frame_vel_input_topic_name, Twist, self.team_frame_command_callback, queue_size=1)
        

        # Swarm footprint polygon publisher
        self.footprint_publish_topic_name = rospy.get_param('~footprint_publish_topic_name', "swarm_footprint")
        self.footprint_pub = rospy.Publisher(self.footprint_publish_topic_name, geometry_msgs.msg.PolygonStamped, queue_size=1)

        # Footprint publish timer
        self.footprint_update_rate = 3.0
        # rospy.Timer(rospy.Duration(1.0 / self.footprint_update_rate), self.publish_formation_footprint_polygon)


        # TF Broadcasts are sent at 30Hz.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        tf_update_rate = 30.0
        rospy.Timer(rospy.Duration(1.0 / tf_update_rate), self.publish_tf_frames)


    def robot_enable_changer(self, msg) -> None:
        number=msg.data
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

    def robot_frame_command_callback(self, msg: Twist, robot_idx: int) -> None:
        if(self.enabled_robots[robot_idx]):
            # Move the position of robot robot_idx in the world frame
            dt = self.get_timestep(self.robot_frame_command_topics[robot_idx])

            qd_world = np.zeros((3,1))
            qd_world[0, 0] = msg.linear.x
            qd_world[1, 0] = msg.linear.y
            qd_world[2, 0] = msg.angular.z

            # robots_xyt is in the swarm frame
            qd_swarm = rot_mat_3d(-self.swarm_xyt[2]).dot(qd_world)
            self.robots_xyt[:,robot_idx] = self.robots_xyt[:,robot_idx] + dt * qd_swarm.flatten()
            #print(self.robots_xyt)

    def team_command_callback(self, msg: Twist, team_id: int) -> None:
        '''Moves the physical team with safe velocities for the robots.

        Checks 
        '''
        enabled_assigned_robots = [
            self.enabled_robots[id] for id in self.assigned_robot_indices[team_id]
        ]
        if sum(enabled_assigned_robots) == 0:
            return

        dt = self.get_timestep(self.team_command_topics[team_id])
        if dt == 0: # Exceeded MAX_TIMESTEP
            self.v_robots_prev *= 0.
            return

        v_desired = np.zeros((3,1))
        v_desired[0, 0] = msg.linear.x
        v_desired[1, 0] = msg.linear.y
        v_desired[2, 0] = msg.angular.z


        enabled_index = np.where(enabled_assigned_robots)[0]

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

    def team_frame_command_callback(self, msg: Twist, team_id: int) -> None:
        '''Moves the team frame without moving the robots.

        Moves the team frame, and moves the robots w.r.t. the team frame
        in the opposite direction
        '''
        dt = self.get_timestep("desired_swarm_frame_velocity")

        qd_world = np.zeros((3,1))
        qd_world[0, 0] = msg.linear.x
        qd_world[1, 0] = msg.linear.y
        qd_world[2, 0] = msg.angular.z

        qd_swarm = rot_mat_3d(-self.swarm_xyt[2, 0]).dot(qd_world)

        q_world_delta = dt * qd_world
        q_swarm_delta = dt * qd_swarm

        self.swarm_xyt = self.swarm_xyt + q_world_delta

        #  np.diag([1., 1., 0.]).
        self.robots_xyt = rot_mat_3d(-q_swarm_delta[2,0]).dot(self.robots_xyt  - q_swarm_delta )

    def tf_changer_callback(self, msg: PoseStamped) -> None:
        '''Changes the robot's frame to the message values.'''
        if(msg.header.frame_id in self.virtual_robot_frame_names):
            idx = self.virtual_robot_frame_names.index(msg.header.frame_id)

            # Get the orientation value and convert to roll-pitch-yaw.
            orientations = [
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w
            ]
            _, _, yaw = tf_conversions.transformations.euler_from_quaternion(
                orientations
            )
            
            # Sanity check.
            rospy.loginfo(f"Changing tf frame {msg.header.frame_id} "
                          f"from {self.robots_xyt[:, idx]} to "
                          f"[{msg.pose.position.x}, {msg.pose.position.y}, "
                          f"{yaw}]."
            )

            # Set the robot's xyt to the message pose.
            self.robots_xyt[0,idx] = msg.pose.position.x
            self.robots_xyt[1,idx] = msg.pose.position.y
            self.robots_xyt[2,idx] = yaw

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
                tf_robot_i = xyt2TF(self.robots_xyt[:,i], "swarm_frame", self.virtual_robot_frame_names[i])
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

    def request_fleet_information(self):
        '''Requests the information about all robots in the fleet.

        The information includes the total number of robots in the fleet,
        robot names, command topics, and frame names.
        '''
        rospy.wait_for_service('fleet_information_service')
        try:
            request = FleetInformationRequest()
            fleet_information = rospy.ServiceProxy(
                "fleet_information_service",
                FleetInformation
            )
            response = fleet_information(request)

            # self.robot_names = response.robot_names
            # self.robot_command_topics = response.robot_command_topics
            self.robot_frame_command_topics = response.robot_frame_command_topics
            # self.real_robot_frame_names = response.real_robot_frame_names
            self.virtual_robot_frame_names = response.virtual_robot_frame_names
            self.state_publish_topics = response.robot_desired_state_topics

            # self.node_names = []
            # for robot in range(self.fleet_size):
            #     self.node_names.append(response.robot_node_names[robot].string_list)

            self.tf_changer_topic = response.tf_changer_topic
            self.robot_enable_status_topic = response.robot_enable_status_topic
            self.fleet_size = response.fleet_size
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Fleet information request failed: {e}.")

    def request_assigned_robot_information(self, ticket_id: int) -> Tuple:
        '''Requests info about the robots assigned to the ticket.'''
        rospy.wait_for_service('robot_assignments_service')
        try:
            request = RobotAssignmentsRequest()
            request.ticket_id = ticket_id
            robot_assignments = rospy.ServiceProxy(
                'robot_assignments_service',
                RobotAssignments
            )
            response = robot_assignments(request)

            num_assigned_robots = response.num_assigned_robots
            assigned_robot_ids = response.assigned_robot_ids

            # Robot IDs start at 1, so subtract 1 to get the indices for
            # accessing their info from the robot info lists.
            assigned_robot_indices = [id-1 for id in assigned_robot_ids]

            team_id = response.team_id
            team_command_topic = response.team_command_topic
            team_frame_command_topic = response.team_frame_command_topic
            team_footprint_topic = response.team_footprint_topic
            team_tf_frame_name = response.team_tf_frame_name

            return num_assigned_robots, assigned_robot_ids,\
                assigned_robot_indices, team_id, team_command_topic,\
                team_frame_command_topic, team_footprint_topic,\
                team_tf_frame_name
        except rospy.ServiceException as e:
            rospy.logerr(f"{log_tag}: Robot assignment request failed: {e}.")

    def update_team_information(self):
        '''Runs often to check.'''
        self.request_ticket_list()
        current_team_ids = []
        for ticket_id in self.ongoing:
            num_assigned_robots, assigned_robot_ids,\
            assigned_robot_indices, team_id, team_command_topic,\
            team_frame_command_topic, team_footprint_topic,\
            team_tf_frame_name = self.request_assigned_robot_information(ticket_id)

            current_team_ids.append(team_id)

            if team_id not in self.team_poses:
                self.team_sizes[team_id] = num_assigned_robots
                self.team_poses[team_id] = np.zeros((3,1))
                self.assigned_robot_ids[team_id] = assigned_robot_ids
                self.assigned_robot_indices[team_id] = assigned_robot_indices
                self.team_command_topics[team_id] = team_command_topic
                self.team_frame_command_topics[team_id] = team_frame_command_topic
                self.team_footprint_topics[team_id] = team_footprint_topic
                self.team_tf_frame_names[team_id] = team_tf_frame_name

                # Create subscribers and publisher.
                self.team_command_subs[team_id] = rospy.Subscriber(
                    team_command_topic, Twist,
                    self.team_command_callback, team_id,
                    queue_size=1,
                )
                self.team_frame_command_subs[team_id] = rospy.Subscriber(
                    team_frame_command_topic, Twist,
                    self.team_frame_command_callback, team_id,
                    queue_size=1,
                )
                # Team footprint polygon publisher.
                self.team_footprint_pubs[team_id] = rospy.Publisher(
                    team_footprint_topic, PolygonStamped,
                    queue_size=1
                )
                # Set the footprint publish timer.
                self.team_footprint_pub_timers[team_id] = rospy.Timer(
                    rospy.Duration(1.0/self.footprint_update_rate),
                    self.publish_formation_footprint_polygon
                )

        # Go through the team_id's we have saved. If there are any that are
        # no longer current, remove them.
        for team_id in self.team_poses.keys():
            if team_id not in current_team_ids:
                self.remove_team(team_id)

    def remove_team(self, team_id: int):
        '''Removes the team from the data structures.'''
        if team_id in self.team_command_topics:
            del(self.team_command_topics[team_id])
        if team_id in self.team_frame_command_topics:
            del(self.team_frame_command_topics[team_id])
        if team_id in self.team_footprint_topics:
            del(self.team_footprint_topics[team_id])
        if team_id in self.team_sizes:
            del(self.team_sizes[team_id])
        if team_id in self.assigned_robot_ids:
            del(self.assigned_robot_ids[team_id])
        if team_id in self.assigned_robot_indices:
            del(self.assigned_robot_indices[team_id])
        if team_id in self.team_poses:
            del(self.team_poses[team_id])

        # Unregister the publisher and subscribers.
        if team_id in self.team_command_subs:
            self.team_command_subs[team_id].unregister()
            del(self.team_command_subs[team_id])
        if team_id in self.team_frame_command_subs:
            self.team_frame_command_subs[team_id].unregister()
            del(self.team_frame_command_subs[team_id])
        if team_id in self.team_footprint_pubs:
            self.team_footprint_pubs[team_id].unregister()
            del(self.team_footprint_pubs[team_id])
        if team_id in self.team_footprint_pub_timers:
            self.team_footprint_pub_timers[team_id].shutdown()


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
