#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: early_progress_parser
Description:
    - TODO
    
Parameters:
    - TODO
Subscribes to:
    - NONE
Publishes to:
    - NONE

Broadcasts to:
    - NONE
"""
import sys
import os

import rospy

import numpy as np
import pandas as pd
import math


class Parser():
    def __init__(self):
        rospy.init_node('early_progress_parser', anonymous=True)


        # Path to input data file
        self.input_data_file = rospy.get_param("~input_data_file")
        self.input_data_file = os.path.expanduser(self.input_data_file)

        self.remove_same_rows = rospy.get_param("~remove_same_rows", True)

        # Read the input data file
        df = pd.read_csv(self.input_data_file)
        self.input = df.values.tolist()
        self.num_times = len(self.input)
        
        # Read the workstation names that are expected to be seen in the input file
        self.ws_names = rospy.get_param("~ws_names")
        self.num_ws =  len(self.ws_names)

        # Read the paths to the hardcoded pose files
        self.hardcoded_data_files = rospy.get_param("~hardcoded_data_files")

        # Create a dictionary with keys as the ws_names to store the hardcoded poses
        self.hardcoded_poses = {}
        for i in range(self.num_ws):
            ws = self.ws_names[i]

            hardcoded_data_file = os.path.expanduser(self.hardcoded_data_files[i])
            df = pd.read_csv(hardcoded_data_file)
            hardcoded_data = df.values.tolist()

            self.hardcoded_poses[ws] = hardcoded_data

        # Get file to export the mapping
        self.export_data_file_name = rospy.get_param("~export_data_file_name")
        self.export_data_directory = rospy.get_param("~export_data_directory")
        
        # Create an empty list to export mapped poses
        self.to_export = []
        
        self.parse()

        # print(self.to_export)

        self.export()

        rospy.loginfo("Parsing is completed successfully. Check: '" + self.export_data_file_name + " in " + self.export_data_directory + "' for the exported file.")
        reason = "Parsing is completed successfully."
        rospy.signal_shutdown(reason)
        # sys.exit()


    def parse(self):
        for i in range(self.num_times):
            row_export = []
            # Read row
            row  = self.input[i] 

            if self.remove_same_rows:
                if i > 0:
                    prev_row = self.input[i-1]
                    if row[1:] == prev_row[1:]:
                        continue

            t = row[0] # 1st element in a row is assumed to be the time stamp of the states

            row_export.append(t)

            robot_ws = row[1:]
            num_robots = len(robot_ws)

            for rob_i in range(num_robots):
                ws_name = robot_ws[rob_i]
                # find hardcoded pose for that robot and that workstation
                x_y_th = self.hardcoded_poses[ws_name][rob_i][1:4]

                row_export.append(x_y_th[0]) # x
                row_export.append(x_y_th[1]) # y
                row_export.append(x_y_th[2]) # theta

            # Append the row to be exported            
            self.to_export.append(row_export)

    def export(self):
        # Create column names
        column_names = ["t"]
        for i in range(len(self.input[0])-1):
            column_names.extend(["x","y","th"])

        df = pd.DataFrame(self.to_export, columns =column_names)
        # print(df)

        # os.makedirs(self.export_data_directory)  
        df.to_csv(self.export_data_directory+"/"+self.export_data_file_name, index=False, header=True) 

if __name__ == '__main__':
    parser = Parser()
    rospy.spin()