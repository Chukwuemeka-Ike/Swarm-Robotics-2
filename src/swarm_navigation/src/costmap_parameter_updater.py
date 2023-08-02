import rospy
from dynamic_reconfigure.client import Client
from rospy.exceptions import ROSException
import time

from shapely.geometry import Point, Polygon, LinearRing
from shapely.ops import nearest_points

import math

class CostmapParameterUpdater:
    def __init__(self, 
                 planner_name = "global_planner",
                 costmap_name = "costmap",
                 inflation_layer_name = "inflater_layer",
                 service_wait_timeout = 2,
                 num_trials = 5,
                 trial_intervals = 0.5
                 ):
        
        # print(planner_name)
        # print(costmap_name)
        # print(inflation_layer_name)
        # print(service_wait_timeout)
        # print(num_trials)
        # print(trial_intervals)
        
        self.costmap_client = "/" + planner_name + "/" + costmap_name
        self.costmap_service = self.costmap_client + "/set_parameters"
        
        self.inflation_layer_client = "/" + planner_name + "/" + costmap_name + "/" + inflation_layer_name
        self.inflation_layer_service = self.inflation_layer_client + "/set_parameters"

        self.service_wait_timeout = service_wait_timeout # seconds
        self.num_trials = num_trials
        self.trial_intervals = trial_intervals # seconds


    # Private functions
    def set_footprint(self, footprint):
        new_config = {"footprint": footprint}
    
        success = True

        # Create a client for the global costmap
        try:
            rospy.wait_for_service(self.costmap_service, timeout=self.service_wait_timeout)
            global_client = Client(self.costmap_client)
            global_client.update_configuration(new_config)
            
            updated = False
            for _ in range(self.num_trials):
                time.sleep(self.trial_intervals)  # wait for the parameter to be updated
                updated_footprint = rospy.get_param(self.costmap_client + "/footprint")
                if updated_footprint == str(footprint):
                    updated = True
                    break
            if not updated:
                rospy.logerr("Failed to update costmap footprint.")
                success = False
        except (ROSException, Exception) as e:
            rospy.logerr("Failed to create a client for costmap or update its footprint: {}".format(e))
            success = False

        return success

    def set_footprint_padding(self, footprint_padding):
        new_config = {"footprint_padding": footprint_padding}
    
        success = True

        # Create a client for the global costmap
        try:
            rospy.wait_for_service(self.costmap_service, timeout=self.service_wait_timeout)
            global_client = Client(self.costmap_client)
            global_client.update_configuration(new_config)
            
            updated = False
            for _ in range(self.num_trials):
                time.sleep(self.trial_intervals)  # wait for the parameter to be updated
                updated_footprint_padding = rospy.get_param(self.costmap_client + "/footprint_padding")
                if updated_footprint_padding == footprint_padding:
                    updated = True
                    break
            if not updated:
                rospy.logerr("Failed to update costmap footprint_padding.")
                success = False
        except (ROSException, Exception) as e:
            rospy.logerr("Failed to create a client for costmap or update its footprint_padding: {}".format(e))
            success = False

        return success

    def set_inflation_radius(self, r_inf):
        new_config = {"inflation_radius": r_inf}

        success = True

        # Create a client for the global costmap
        try:
            rospy.wait_for_service(self.inflation_layer_service, timeout=self.service_wait_timeout)
            global_client = Client(self.inflation_layer_client)
            global_client.update_configuration(new_config)
            
            updated = False
            for _ in range(self.num_trials):
                time.sleep(self.trial_intervals)  # wait for the parameter to be updated
                updated_radius = rospy.get_param(self.inflation_layer_client + "/inflation_radius")
                if updated_radius == r_inf:
                    updated = True
                    break
            if not updated:
                rospy.logerr("Failed to update costmap inflation radius.")
                success = False
        except (ROSException, Exception) as e:
            rospy.logerr("Failed to create a client for costmap or update its inflation radius: {}".format(e))
            success = False

        return success

    def set_cost_scaling_factor(self,c):
        new_config = {"cost_scaling_factor": c}

        success = True

        # Create a client for the global costmap
        try:
            rospy.wait_for_service(self.inflation_layer_service, timeout=self.service_wait_timeout)
            global_client = Client(self.inflation_layer_client)
            global_client.update_configuration(new_config)
            
            updated = False
            for _ in range(self.num_trials):
                time.sleep(self.trial_intervals)  # wait for the parameter to be updated
                updated_cost_factor = rospy.get_param(self.inflation_layer_client + "/cost_scaling_factor")
                if updated_cost_factor == c:
                    updated = True
                    break
            if not updated:
                rospy.logerr("Failed to update costmap cost scaling factor.")
                success = False
        except (ROSException, Exception) as e:
            rospy.logerr("Failed to create a client for costmap or update its cost scaling factor: {}".format(e))
            success = False

        return success

    # Utility private functions
    def circumscribed_radius(self, polygon_coords):
        # Initialize maximum distance to be negative infinity
        max_distance = -math.inf

        # Iterate through the polygon coordinates
        for coord in polygon_coords:
            # Calculate the distance from the center (0,0) to the current point
            distance = math.sqrt(coord[0]**2 + coord[1]**2)

            # If this distance is greater than the current maximum distance, update the maximum distance
            if distance > max_distance:
                max_distance = distance
        
        # Return the maximum distance found, which is the circumscribed radius
        return max_distance

    def inscribed_radius(self, polygon_coords):
        # Construct LinearRing object
        ring = LinearRing(polygon_coords)

        # Define center point
        center = Point(0, 0)

        # Return the minimum distance from the center to the ring - which is the "inscribed radius"
        return center.distance(ring)

    # Public functions
    def set_cost_scaling_factor_n_inflation_radius_n_padding(self, footprint):
        r_i = self.inscribed_radius(footprint)
        r_c = self.circumscribed_radius(footprint)

        footprint_padding = r_c - r_i

        # As a rule of thumb, inflation radius = 4 x inscribed_radius 
        r_inf = 4.0*r_c

        # As a rule of thumb, cost_scaling_factor = -ln(128/253)/(r_i-r_c) ~= 0.681/(r_c-r_i)
        if (r_c - r_i) > 0.015: 
            c = 0.681/(r_c-r_i)
        else:
            c = 45.0

        # debug info
        rospy.loginfo("New costmap inscribed_radius:     " + str (r_i))
        rospy.loginfo("New costmap circumscribed_radius: " + str (r_c))
        rospy.loginfo("New costmap inflation radius:     " + str (r_inf))
        rospy.loginfo("New costmap cost_scaling_factor:  " + str (c))
        
        success1 = self.set_inflation_radius(r_inf)
        success2 = self.set_cost_scaling_factor(c)
        success3 = self.set_footprint_padding(footprint_padding)

        return (success1 and success2 and success3)

    def update_costmap_parameters(self, footprint):
        success1 = self.set_footprint(footprint)
        success2 = self.set_cost_scaling_factor_n_inflation_radius_n_padding(footprint)

        return (success1 and success2)



if __name__ == "__main__":
    # ------------------------ TEST -----------------------------------------
    rospy.init_node("costmap_parameter_updater")

    planner_name = "global_planner"
    costmap_name = "costmap"
    inflation_layer_name = "inflater_layer"
    service_wait_timeout = 2
    num_trials = 5
    trial_intervals = 0.5

    updater = CostmapParameterUpdater(planner_name,
                                      costmap_name,
                                      inflation_layer_name,
                                      service_wait_timeout,
                                      num_trials,
                                      trial_intervals)


    # Example polygon
    # footprint = [[1.0,-1.0], [1.0,1.0], [-1.0,1.0], [-1.0,-1.0]]
    # footprint = [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
    footprint = [[-0.5, -0.33], [-0.5, 0.33], [0.5, 0.33], [0.5, -0.33]]

    success = updater.update_costmap_parameters(footprint)
    print("success?")
    print("return: " , str(success))
    # ------------------------ TEST -----------------------------------------



