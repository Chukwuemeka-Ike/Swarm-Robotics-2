import rospy
from dynamic_reconfigure.client import Client
from rospy.exceptions import ROSException
import time

from shapely.geometry import Point, Polygon, LinearRing
from shapely.ops import nearest_points

import math

def circumscribed_radius(polygon_coords):
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

def inscribed_radius(polygon_coords):
    # Construct LinearRing object
    ring = LinearRing(polygon_coords)

    # Define center point
    center = Point(0, 0)

    # Return the minimum distance from the center to the ring - which is the "inscribed radius"
    return center.distance(ring)

def set_inflation_radius(r_inf):
    new_config = {"inflation_radius": r_inf}

    service_wait_timeout = 2 # seconds
    num_trials = 5
    trial_intervals = 0.5 # seconds

    # Create a client for the global costmap
    try:
        rospy.wait_for_service("/move_base/global_costmap/inflation/set_parameters", timeout=service_wait_timeout)
        global_client = Client("/move_base/global_costmap/inflation")
        global_client.update_configuration(new_config)
        
        updated = False
        for _ in range(num_trials):
            time.sleep(trial_intervals)  # wait for the parameter to be updated
            updated_radius = rospy.get_param("/move_base/global_costmap/inflation/inflation_radius")
            if updated_radius == r_inf:
                updated = True
                break
        if not updated:
            rospy.logerr("Failed to update global costmap inflation radius.")
    except (ROSException, Exception) as e:
        rospy.logerr("Failed to create a client for global costmap or update its inflation radius: {}".format(e))

    # Create a client for the local costmap
    try:
        rospy.wait_for_service("/move_base/local_costmap/inflation/set_parameters", timeout=service_wait_timeout)
        local_client = Client("/move_base/local_costmap/inflation")
        local_client.update_configuration(new_config)
        
        updated = False
        for _ in range(num_trials):
            time.sleep(trial_intervals)  # wait for the parameter to be updated
            updated_radius = rospy.get_param("/move_base/local_costmap/inflation/inflation_radius")
            if updated_radius == r_inf:
                updated = True
                break
        if not updated:
            rospy.logerr("Failed to update local costmap inflation radius.")
    except (ROSException, Exception) as e:
        rospy.logerr("Failed to create a client for local costmap or update its inflation radius: {}".format(e))

def set_cost_scaling_factor(c):
    new_config = {"cost_scaling_factor": c}

    service_wait_timeout = 2 # seconds
    num_trials = 5
    trial_intervals = 0.5 # seconds

    # Create a client for the global costmap
    try:
        rospy.wait_for_service("/move_base/global_costmap/inflation/set_parameters", timeout=service_wait_timeout)
        global_client = Client("/move_base/global_costmap/inflation")
        global_client.update_configuration(new_config)
        
        updated = False
        for _ in range(num_trials):
            time.sleep(trial_intervals)  # wait for the parameter to be updated
            updated_cost_factor = rospy.get_param("/move_base/global_costmap/inflation/cost_scaling_factor")
            if updated_cost_factor == c:
                updated = True
                break
        if not updated:
            rospy.logerr("Failed to update global costmap cost scaling factor.")
    except (ROSException, Exception) as e:
        rospy.logerr("Failed to create a client for global costmap or update its cost scaling factor: {}".format(e))

    # Create a client for the local costmap
    try:
        rospy.wait_for_service("/move_base/local_costmap/inflation/set_parameters", timeout=service_wait_timeout)
        local_client = Client("/move_base/local_costmap/inflation")
        local_client.update_configuration(new_config)
        
        updated = False
        for _ in range(num_trials):
            time.sleep(trial_intervals)  # wait for the parameter to be updated
            updated_cost_factor = rospy.get_param("/move_base/local_costmap/inflation/cost_scaling_factor")
            if updated_cost_factor == c:
                updated = True
                break
        if not updated:
            rospy.logerr("Failed to update local costmap cost scaling factor.")
    except (ROSException, Exception) as e:
        rospy.logerr("Failed to create a client for local costmap or update its cost scaling factor: {}".format(e))

def set_cost_scaling_factor_n_inflation_radius(footprint):
    r_i = inscribed_radius(footprint)
    r_c = circumscribed_radius(footprint)

    # As a rule of thumb, inflation radius = 4 x inscribed_radius 
    r_inf = 4.0*r_i

    # As a rule of thumb, cost_scaling_factor = -ln(128/253)/(r_i-r_c) ~= 0.681/(r_c-r_i)
    c = 0.681/(r_c-r_i)

    # debug info
    print("New costmap inscribed_radius:     ", str (r_i))
    print("New costmap circumscribed_radius: ", str (r_c))
    print("New costmap inflation radius:     ", str (r_inf))
    print("New costmap cost_scaling_factor:  ", str (c))
    
    set_inflation_radius(r_inf)
    set_cost_scaling_factor(c)

def set_footprint(footprint):
    new_config = {"footprint": footprint}
    
    service_wait_timeout = 2 # seconds
    num_trials = 5
    trial_intervals = 0.5 # seconds

    # Create a client for the global costmap
    try:
        rospy.wait_for_service("/move_base/global_costmap/set_parameters", timeout=service_wait_timeout)
        global_client = Client("/move_base/global_costmap")
        global_client.update_configuration(new_config)
        
        updated = False
        for _ in range(num_trials):
            time.sleep(trial_intervals)  # wait for the parameter to be updated
            updated_footprint = rospy.get_param("/move_base/global_costmap/footprint")
            if updated_footprint == str(footprint):
                updated = True
                break
        if not updated:
            rospy.logerr("Failed to update global costmap footprint.")
    except (ROSException, Exception) as e:
        rospy.logerr("Failed to create a client for global costmap or update its footprint: {}".format(e))

    # Create a client for the local costmap
    try:
        rospy.wait_for_service("/move_base/local_costmap/set_parameters", timeout=service_wait_timeout)
        local_client = Client("/move_base/local_costmap")
        local_client.update_configuration(new_config)

        updated = False
        for _ in range(num_trials):
            time.sleep(trial_intervals)  # wait for the parameter to be updated
            updated_footprint = rospy.get_param("/move_base/local_costmap/footprint")
            if updated_footprint == str(footprint):
                updated = True
                break
        if not updated:
            rospy.logerr("Failed to update local costmap footprint.")
    except (ROSException, Exception) as e:
        rospy.logerr("Failed to create a client for local costmap or update its footprint: {}".format(e))

def main():
    rospy.init_node("costmap_parameter_updater")

    # Example polygon
    # footprint = [[1.0,-1.0], [1.0,1.0], [-1.0,1.0], [-1.0,-1.0]]
    footprint = [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
    # footprint = [[-0.5, -0.33], [-0.5, 0.33], [0.5, 0.33], [0.5, -0.33]]

    set_footprint(footprint)
    set_cost_scaling_factor_n_inflation_radius(footprint)

if __name__ == "__main__":
    main()



