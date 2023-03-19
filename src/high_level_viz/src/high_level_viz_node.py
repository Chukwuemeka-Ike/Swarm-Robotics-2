#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: high_level_viz_node
Description:
    - TODO
    
Parameters:
    - TODO
Subscribes to:
    - TODO
Publishes to:
    - TODO

Broadcasts to:
    - NONE
"""
import os

import rospy

import numpy as np
import pandas as pd
import math

import visualization_msgs.msg  # Marker
import gazebo_msgs.msg # ModelState


class RobotStateVisualizer():
    def __init__(self):
        rospy.init_node('high_level_visualizer', anonymous=True)

        self.model_states_topic_name = rospy.get_param("~model_states_topic_name", "/gazebo/set_model_state")
        self.model_world_frame = rospy.get_param("~model_world_frame", "map")

        # Path to data file
        self.path_to_data_file = rospy.get_param("~path_to_data_file", "~/catkin_ws_swarm2/src/high_level_viz/data/sample.csv")
        self.path_to_data_file = os.path.expanduser(self.path_to_data_file)

        # Read the data file ignoring the first row since it is the header row
        # df = pd.read_csv(self.path_to_data_file, skiprows=[0])
        df = pd.read_csv(self.path_to_data_file)
        self.times_n_states_all = df.values.tolist()
        self.num_times = len(self.times_n_states_all)
        self.time_itr = 0

        # Read model names to be updated
        self.model_names = rospy.get_param("~model_names")
        self.num_models =  len(self.model_names)

        # Time marker related parameters
        self.time_marker_topic_name = rospy.get_param("~time_marker_topic_name", "time_marker")

        self.time_marker_frame = rospy.get_param("~time_marker_frame", "map")
        self.time_marker_x = rospy.get_param("~time_marker_x", 0.0)
        self.time_marker_y = rospy.get_param("~time_marker_y", 0.0)
        self.time_marker_z = rospy.get_param("~time_marker_z", 0.0)
        self.time_marker_scale = rospy.get_param("~time_marker_scale", 1.0)

        # Publishers
        self.pub_time_marker = rospy.Publisher(self.time_marker_topic_name, visualization_msgs.msg.Marker, queue_size=1) 
        self.pub_model_states = rospy.Publisher(self.model_states_topic_name, gazebo_msgs.msg.ModelState, queue_size=self.num_models) 



        # Time to display each state in the data file
        self.pub_time_interval = rospy.get_param("~pub_time_interval", 1.0)

        # Start publishing
        rospy.Timer(rospy.Duration(self.pub_time_interval), self.visualizer)


    def visualizer(self, event=None):
        if (self.time_itr < self.num_times):
            # Read row
            row  = self.times_n_states_all[self.time_itr] 

            if (len(row) >= (1 + 3*self.num_models)):
                t = row[0] # 1st element in a row is assumed to be the time stamp of the states

                # Update time stamp marker
                self.update_time_marker(t)

                # Update each pose of each model
                for i in range(self.num_models):
                    # Model name
                    model_name = self.model_names[i]
                    # Remaining elements (row[1:]) has the (x,y,theta) info of each model 
                    x,y,th = row[(1+3*i) : (1+3*i+3)]
                    # we assumed th is specified in degrees, convert it to radians
                    th = np.deg2rad(th)

                    self.update_model_state(model_name,x,y,th)

            else:
                rospy.logerr("The given csv file does not have as many data as the specified model names!")
            
            # increase time_itr by one
            self.time_itr = self.time_itr + 1
        else:
            # reset to start over
            self.time_itr = 0
            rospy.loginfo("Visualized all time states given in the csv file. Starting over..")

    def update_model_state(self,name,x,y,th):
        msg = gazebo_msgs.msg.ModelState()

        msg.reference_frame = self.model_world_frame

        msg.model_name = name

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # axis: 0,0,1 (z-axis)
        # angle: th
        # corresponding quaternion: w = cos(th/2), (x,y,z) = sin(th/2)*[0,0,1]
        # hence; 

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(th/2.0)
        msg.pose.orientation.w = math.cos(th/2.0)

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub_model_states.publish(msg)

    def update_time_marker(self,t):
        # t is given in seconds, convert it to a nice string
        hours = t // 3600
        minutes = (t % 3600) // 60
        seconds = t % 60
        t_str = "%dh %dm %ds" % (hours, minutes, seconds)

        # Publish t_str as marker to RVIZ
        m = visualization_msgs.msg.Marker()

        m.header.frame_id = self.time_marker_frame
        m.header.stamp = rospy.Time.now()

        m.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        m.id = 0
        m.action = m.ADD

        # only z scale is used
        m.scale.z = self.time_marker_scale

        m.color.a = 1.
        m.color.r = 1.
        m.color.g = 1.
        m.color.b = 1.

        m.pose.position.x = self.time_marker_x
        m.pose.position.y = self.time_marker_y
        m.pose.position.z = self.time_marker_z

        m.text = t_str

        self.pub_time_marker.publish(m)

if __name__ == '__main__':
    robotStateVisualizer = RobotStateVisualizer()
    rospy.spin()