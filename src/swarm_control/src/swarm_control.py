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
import time
from typing import Tuple

import numpy as np
import rospy
import tf2_ros
import tf_conversions # quaternion stuff
import shapely
import shapely.geometry
import shapely.affinity

from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped,\
    TransformStamped, Twist

from arm_msgs.msg import RobotEnableStatus
from arm_msgs.srv import FleetInformation, FleetInformationRequest,\
        RobotAssignments, RobotAssignmentsRequest,\
        TicketList, TicketListRequest
from swarm_msgs.msg import State2D
from utilities.safe_swarm_controller import safe_motion_controller


# Velocity commands will only be considered if they are
# spaced closer than MAX_TIMESTEP.
MAX_TIMESTEP = 0.1
log_tag = "Swarm Controller"


class Swarm_Control:
    '''.'''
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
        self.robot_enable_status = []

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

            self.robot_enable_status.append(True)

        # Subscriber for the robot enable status.
        rospy.Subscriber(
            self.robot_enable_status_topic, RobotEnableStatus,
            self.robot_enable_changer_callback, queue_size=5
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

        # Team footprint update rate.
        self.footprint_update_rate = 3.0

        # TF Broadcasts are sent at 30Hz.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        tf_update_rate = 30.0
        rospy.Timer(rospy.Duration(1.0 / tf_update_rate), self.publish_tf_frames)

        # Timer for updating team information.
        rospy.Timer(
            rospy.Duration(1.0 / 2.0),
            self.update_team_information
        )

    def shutdown(self) -> None:
        '''.'''
        rospy.loginfo(f"{log_tag}: Node shutdown.")

    def robot_frame_command_callback(self, msg: Twist, robot_idx: int) -> None:
        '''Adjusts the robot's virtual frame w.r.t world frame.

        Args:
            msg: Twist message containing the desired change.
            robot_idx: index of the robot in the lists of robot-level info.
        '''
        if(self.robot_enable_status[robot_idx]):
            dt = self.get_timestep(self.robot_frame_command_topics[robot_idx])

            # Get which team the robot belongs to.
            robot_team_id = -1
            for team_id, team_member_indices in self.assigned_robot_indices.items():
                if robot_idx in team_member_indices:
                    robot_team_id = team_id

            # Only update if we found the robot in a team.
            if robot_team_id != -1:
                qd_world = np.zeros((3,1))
                qd_world[0, 0] = msg.linear.x
                qd_world[1, 0] = msg.linear.y
                qd_world[2, 0] = msg.angular.z

                # robots_xyt is in the team frame.
                qd_swarm = rot_mat_3d(
                    -self.team_poses[robot_team_id][2]
                ).dot(qd_world)
                self.robots_xyt[:,robot_idx] = self.robots_xyt[:, robot_idx] +\
                        dt * qd_swarm.flatten()
                print(self.robots_xyt)

    def team_command_callback(self, msg: Twist, team_id: int) -> None:
        '''Moves the physical team with safe velocities for the enabled robots.

        Checks that the max timestep hasn't been exceeded, then calculates
        safe motion for the enabled robots in that team.

        Args:
            msg: Twist message containing the desired change.
            team_id: ID of the team the message is intended for.
        '''
        assigned_robot_enable_status = [
            self.robot_enable_status[i] for i in self.assigned_robot_indices[team_id]
        ]
        if sum(assigned_robot_enable_status) == 0:
            return

        dt = self.get_timestep(self.team_command_topics[team_id])
        if dt == 0: # Exceeded MAX_TIMESTEP
            self.v_robots_prev *= 0.
            return

        # Get the desired team velocity.
        v_desired = np.zeros((3,1))
        v_desired[0, 0] = msg.linear.x
        v_desired[1, 0] = msg.linear.y
        v_desired[2, 0] = msg.angular.z

        # Get the indices of the robots that are both assigned and enabled.
        # First get the idx of True values in assigned list.
        enabled_index = np.where(assigned_robot_enable_status)[0]
        # Use the idx from above to get the absolute idx in the robot-level vars.
        enabled_index = [
            self.assigned_robot_indices[team_id][idx] for idx in enabled_index
        ]

        # Position and angle of each robot in the team frame.
        p_i_mat = self.robots_xyt[:2, enabled_index]
        theta_vec = self.robots_xyt[2, enabled_index]

        # Calculate safe inputs for the enabled robots in the team.
        v_i_world, v_i_rob, xyt_i, _, team_next_pose = safe_motion_controller(
            v_desired, self.theta_scale, p_i_mat, theta_vec,
            self.v_max[:, enabled_index], self.a_max[:, enabled_index], dt,
            sum(assigned_robot_enable_status),
            self.v_robots_prev[:, enabled_index], self.team_poses[team_id]
        )

        # Don't update self.robots_xyt, since that's in the team frame.
        # Set the new previous velocities.
        self.v_robots_prev[:, enabled_index] = v_i_rob
        # Update the team pose.
        self.team_poses[team_id] = team_next_pose

        # Send desired state to each enabled robot in the team.
        for i in range(len(enabled_index)):
            # Absolute index for the robot.
            robot_idx = enabled_index[i]

            s = State2D()

            s.pose.x = xyt_i[0, i]
            s.pose.y = xyt_i[1, i]
            s.pose.theta = xyt_i[2, i]

            s.twist.linear.x = v_i_world[0, i]
            s.twist.linear.y = v_i_world[1, i]
            s.twist.angular.z = v_i_world[2, i]

            self.robot_command_pubs[robot_idx].publish(s)

    def team_frame_command_callback(self, msg: Twist, team_id: int) -> None:
        '''Moves the team frame without moving the robots.

        Moves the team frame, and moves the robots w.r.t. the team frame
        in the opposite direction.

        Args:
            msg: Twist message containing the desired change.
            team_id: ID of the team the message is intended for.
        '''
        dt = self.get_timestep(self.team_command_topics[team_id])

        # 
        qd_world = np.zeros((3,1))
        qd_world[0, 0] = msg.linear.x
        qd_world[1, 0] = msg.linear.y
        qd_world[2, 0] = msg.angular.z

        # 
        qd_team = rot_mat_3d(
                -self.team_poses[team_id][2, 0]
            ).dot(qd_world)

        q_world_delta = dt * qd_world
        q_team_delta = dt * qd_team

        # Adjust the team's pose w.r.t the world.
        self.team_poses[team_id] = self.team_poses[team_id] + q_world_delta

        # Adjust the team's individual robot's poses w.r.t the team's pose in
        # the opposite direction.
        team_members = self.assigned_robot_indices[team_id]
        self.robots_xyt[:, team_members] = rot_mat_3d(
                -q_team_delta[2, 0]
            ).dot(self.robots_xyt[:, team_members]  - q_team_delta)

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
            self.robots_xyt[0, idx] = msg.pose.position.x
            self.robots_xyt[1, idx] = msg.pose.position.y
            self.robots_xyt[2, idx] = yaw

    def get_timestep(self, topic_name: str) -> None:
        '''Gets the time-step of the last message on that topic'''
        current_time = time.time()
        if topic_name in self.last_timestep_requests:
            dt = current_time - self.last_timestep_requests[topic_name]
            self.last_timestep_requests[topic_name] = current_time
            if dt > MAX_TIMESTEP:
                dt = 0.0
            return dt
        else:
            self.last_timestep_requests[topic_name] = current_time
            return 0.0

    def publish_tf_frames(self, _: rospy.timer.TimerEvent) -> None:
        '''Publishes the TF frames for all teams and their enabled members.'''
        # Publish the transforms for every team wrt map frame.
        for team_id, team_pose in self.team_poses.items():
            team_transform = xyt2TF(
                team_pose, "map", self.team_tf_frame_names[team_id]
            )
            self.tf_broadcaster.sendTransform(team_transform)

        # Publish the transforms for every enabled robot wrt their team frame.
        for team_id, team_member_indices in self.assigned_robot_indices.items():
            for robot_idx in team_member_indices:
                if self.robot_enable_status[robot_idx] == True:
                    robot_transform = xyt2TF(
                        self.robots_xyt[:, robot_idx],
                        self.team_tf_frame_names[team_id],
                        self.virtual_robot_frame_names[robot_idx]
                    )
                    self.tf_broadcaster.sendTransform(robot_transform)                

    def publish_team_footprint(
            self, team_id: int, _: rospy.timer.TimerEvent
        ) -> None:
        '''Publishes the footprint of the enabled robots in a team.

        Args:
            team_id: the ID of the team.
            _: rospy timer event.
        '''
        # If no robots in the team are enabled, do nothing.
        assigned_robot_enable_status = [
            self.robot_enable_status[i] for i in self.assigned_robot_indices[team_id]
        ]
        if sum(assigned_robot_enable_status) == 0:
            return

        # Find the points in the active formation.
        coords = []
        coords.append([0.0,0.0]) # Include the location of the team frame.
        default_footprint = [
            [0.345, -0.260], [0.345, 0.260], [-0.345, 0.260], [-0.345, -0.260]
        ]
        for point in default_footprint:
            coords.append(point)

        for robot_idx in self.assigned_robot_indices[team_id]:
            if self.robot_enable_status[robot_idx] == True:
                # Robot pose in team frame.
                xyt = self.robots_xyt[:, robot_idx].flatten()

                # Get the footprint.
                robot_pts = self.footprints[robot_idx] # Nx2
                # Convert to shapely multi point object.
                robot_pts_multi_pt = shapely.geometry.MultiPoint(robot_pts)
                # Rotate.
                robot_pts_multi_pt = shapely.affinity.rotate(
                    robot_pts_multi_pt, xyt[2], origin=(0,0), use_radians=True
                )
                # Translate.
                robot_pts_multi_pt = shapely.affinity.translate(
                    robot_pts_multi_pt, xoff=xyt[0], yoff=xyt[1]
                )
                for point in robot_pts_multi_pt.geoms:
                    coords.append(list(point.coords[0]))

        # Find the convex hull of the points with shapely.
        # NOTE: convex_hull will return a Polygon if your hull contains more
        # than two points. If it only contains two points, you'll get a
        # LineString, and if it only contains one point, you'll get a Point.
        multi_pt = shapely.geometry.MultiPoint(coords)
        hull = multi_pt.convex_hull

        # Convert the hull points to ROS Point32 messages if it is a Polygon.
        polygon = []
        if hull.geom_type == 'Polygon':
            for pt in list(hull.exterior.coords):
                msg = Point32()
                msg.x = pt[0]
                msg.y = pt[1]
                msg.z = 0  # assuming 2D coordinates.
                polygon.append(msg)
        else:
            rospy.loginfo("Not enough points to form a polygon.")
            return

        # Prepare the msg and publish on that team's publisher.
        msg = PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.team_tf_frame_names[team_id]
        msg.polygon.points = polygon
        self.team_footprint_pubs[team_id].publish(msg)

    def robot_enable_changer_callback(self, msg: RobotEnableStatus) -> None:
        '''Changes the enable status of the robots based on the message.

        Args:
            msg: RobotEnableStatus containing enabled_ids and disabled_ids.
                    Each message is team-specific and contains the true
                    robot IDs.
        '''
        enabled_ids = msg.enabled_ids
        disabled_ids = msg.disabled_ids

        # Set the enabled IDs to True.
        for robot_id in enabled_ids:
            # Indices start at 0, but ID's start at 1.
            robot_idx = robot_id - 1
            self.robot_enable_status[robot_idx] = True

        # Set the disabled IDs to False.
        for robot_id in disabled_ids:
            # Indices start at 0, but ID's start at 1.
            robot_idx = robot_id - 1
            self.robot_enable_status[robot_idx] = False
        
        rospy.loginfo(f"{log_tag}: Robot enable status: "
                      f"{self.robot_enable_status}"
        )

    def request_ticket_list(self) -> None:
        '''Requests the current ticket list from the ticket_service.'''
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

    def request_fleet_information(self) -> None:
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

    def update_team_information(self, _: rospy.timer.TimerEvent) -> None:
        '''Runs often to check.'''
        self.request_ticket_list()
        current_team_ids = []
        for ticket_id in self.ongoing:
            num_assigned_robots, assigned_robot_ids,\
            assigned_robot_indices, team_id, team_command_topic,\
            team_frame_command_topic, team_footprint_topic,\
            team_tf_frame_name = self.request_assigned_robot_information(ticket_id)

            current_team_ids.append(team_id)

            # Create the team ID and all necessary variables if it doesn't
            # exist yet.
            if team_id not in self.team_poses:
                print(f"Adding team {team_id}")
                self.team_sizes[team_id] = num_assigned_robots
                self.team_poses[team_id] = np.zeros((3,1))
                self.assigned_robot_ids[team_id] = assigned_robot_ids
                self.assigned_robot_indices[team_id] = assigned_robot_indices

                self.team_command_topics[team_id] = team_command_topic
                self.team_frame_command_topics[team_id] = team_frame_command_topic
                self.team_footprint_topics[team_id] = team_footprint_topic
                self.team_tf_frame_names[team_id] = team_tf_frame_name

                # Create team subscribers and publisher.
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
                # Lambda used to pass the team ID to the callback.
                self.team_footprint_pub_timers[team_id] = rospy.Timer(
                    rospy.Duration(1.0/self.footprint_update_rate),
                    lambda event: self.publish_team_footprint(
                        team_id, event
                    )
                )
            # Cross-check existing team IDs for changes. The things that could
            # change are related to robot membership, so we update those.
            elif team_id in self.team_poses:
                self.team_sizes[team_id] = num_assigned_robots
                self.assigned_robot_ids[team_id] = assigned_robot_ids
                self.assigned_robot_indices[team_id] = assigned_robot_indices

        print(f"Teams {current_team_ids}.")
        # Go through the team_id's we have saved. If there are any that are
        # no longer current, remove them.
        remove_ids = []
        for team_id in self.team_poses.keys():
            if team_id not in current_team_ids:
                remove_ids.append(team_id)
        for team_id in remove_ids:
            self.remove_team(team_id)

    def remove_team(self, team_id: int):
        '''Removes the team from the team data structures.'''
        # Delete the team's information.
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

        # Shutdown the timer.
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

def xyt2TF(
        xyt: np.ndarray, header_frame_id: str, child_frame_id: str
    ) -> TransformStamped:
    '''Converts a pose to a TF transform message.

    Args: numpy vector [x; y; theta]
    into a tf2_msgs.msg.TFMessage message
        header_frame_id: name of the 
    Returns:
        msg: TF transform message with the frame relationships.
    '''
    xyt = xyt.flatten()
    t = TransformStamped()

    t.header.frame_id = header_frame_id

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
