#!/usr/bin/env python3

import pandas as pd
import rospy
import numpy as np

from swarm_msgs.msg import State2D
from geometry_msgs.msg import Pose2D, Twist
import nav_msgs.msg

import tf_conversions # quaternion stuff

class PathFromCsv:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_from_csv', anonymous=True)
        
        # Get parameters
        # filename = rospy.get_param("~file_name", "zero_path.csv")
        # filename = rospy.get_param("~file_name", "VINYL_716262_path_initial.csv")
        filename = rospy.get_param("~file_name", "VINYL_716262_path.csv")
        # filename = rospy.get_param("~file_name", "VINYL_716262_path_equal_centroid.csv")

        self.pub_rate = rospy.get_param("~pub_rate", 100.0)
        self.waypoint_update_rate = rospy.get_param("~waypoint_update_rate", 50.0)

        self.position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name', "main/swarm_frame/odom")

        self.pose_dist_tolerance = rospy.get_param("~pos_dist_tolerance", 0.05) # 0.5cm
        self.pose_ori_tolerance  = rospy.get_param("~pose_ori_tolerance", np.deg2rad(10)) # 3 deg 


        # Create a publisher for the State2D messages
        self.publisher = rospy.Publisher('/swarm/desired_state', State2D, queue_size=1)
        
        # Read the data from the CSV file
        self.df_path = pd.read_csv(filename)
        
        self.index = 0

        self.waypoint_reached = False

        # Setup timer for publishing
        rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self.publish_callback)

        # Setup timer for waypoint updating
        rospy.Timer(rospy.Duration(1.0 / self.waypoint_update_rate), self.update_waypoint_callback)

        # Subscribe to Filtered position
        rospy.Subscriber(self.position_feedback_topic_name, nav_msgs.msg.Odometry, self.state_feedback_callback, queue_size=1)
    
    def publish_callback(self, event):
        row = self.df_path.iloc[self.index]
        # Create and publish the State2D message
        state2d = State2D()
        state2d.pose = Pose2D(row['x'], row['y'], row['theta'])
        self.publisher.publish(state2d)

    def update_waypoint_callback(self, event):
        if self.waypoint_reached:
            # update index or wrap around
            self.index = (self.index + 1) % len(self.df_path)
            self.waypoint_reached = False

    def state_feedback_callback(self, data):
        orientations = [data.pose.pose.orientation.x, 
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w]
        (roll,pitch,yaw) = tf_conversions.transformations.euler_from_quaternion(orientations)
        
        curr_pos = np.array([ data.pose.pose.position.x, data.pose.pose.position.y])
        curr_ori = yaw # theta

        wayp_pos = np.array([self.df_path.iloc[self.index]['x'], self.df_path.iloc[self.index]['y']])
        wayp_ori = self.df_path.iloc[self.index]['theta']

        pose_dist = np.linalg.norm(curr_pos - wayp_pos)
        pose_ori = abs(curr_ori - wayp_ori)

        if pose_dist <= self.pose_dist_tolerance and pose_ori <= self.pose_ori_tolerance:
            self.waypoint_reached = True


if __name__ == '__main__':
    node = PathFromCsv()
    rospy.spin()






