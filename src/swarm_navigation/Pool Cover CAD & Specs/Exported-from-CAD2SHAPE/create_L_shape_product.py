import csv
import math
import numpy as np

import matplotlib.pyplot as plt

import shapely.geometry
from shapely import LineString

from scipy.interpolate import make_interp_spline

def generate_L_shape_coordinates(radius, 
                                 L_skeleton_long_num_r = 2,
                                 L_skeleton_short_num_r = 2,
                                 num_points=200):
    # Generating the coordinates for the L shape
    coordinates = []

    assert L_skeleton_short_num_r > 1, "Short side should be longer than single radius r" 
    assert L_skeleton_long_num_r > 1, "Long side should be longer than single radius r" 
    assert L_skeleton_long_num_r >= L_skeleton_short_num_r, "Long side should be longer than the short side"

    # normal L shape
    # L_skeleton = LineString([(0,-L_skeleton_short_num_r*radius),(0,0),(L_skeleton_long_num_r*radius,0)])
    # reflected L shape
    L_skeleton = LineString([(0,L_skeleton_short_num_r*radius),(0,0),(L_skeleton_long_num_r*radius,0)])

    L = L_skeleton.buffer(radius, cap_style="round", join_style="round") # dilate
    L = L.buffer(1*radius).buffer(-1*radius) # Smooths the sharp inner corner to radius 

    # If num_points is None, simply return the original boundary coordinates
    if num_points is None:
        coordinates = list(L.boundary.coords)
        coordinates = coordinates[::-1] # reverse the list for counterclockwise order
        return coordinates[:-1] # We return up to -1 to exclude the duplicate point at the end due to periodicity

    # Otherwise, interpolate based on num_points
    # Use the logic of spline interpolation with periodic boundary conditions
    x, y = L.boundary.xy

    # Creating the cumulative distance array, setting the start point as 0
    cumulative_distance = np.zeros(len(x))
    for i in range(1, len(x)):
        cumulative_distance[i] = cumulative_distance[i-1] + np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)

    # Defining the interpolation function for x, y coordinates
    spl = make_interp_spline(cumulative_distance, np.c_[x, y], k=3, bc_type='periodic')

    # Defining new waypoints, spaced evenly along the total path length
    perimeter = cumulative_distance[-1]
    # n_pts = math.ceil(perimeter * num_points)
    n_pts = num_points

    new_distances = np.linspace(0, perimeter, n_pts)
    new_points = spl(new_distances)

    coordinates = list(new_points)
    coordinates = coordinates[::-1] # reverse the list for counterclockwise order
    return coordinates[:-1] # We return up to -1 to exclude the duplicate point at the end due to periodicity

def save_to_csv(radius, filename="VINYL L_SHAPE_1_pt.csv"):
    coordinates = generate_L_shape_coordinates(radius)
    
    # Writing the coordinates to csv
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["_X", "_Y", "ID", "LAYER", "ENTITY"])

        id_value = 121  # Starting ID value for COVER_HEM
        for x, y in coordinates:
            writer.writerow([f"{x:.5f}", f"{y:.5f}", id_value, "COVER_HEM", "LWPOLYLINE"])

        # Writing the alignment line
        alignment_id = 119
        writer.writerow([f"{5*radius:.5f}", "0.00000", alignment_id, "ALIGNMENT_LINE", "LINE"])  # Right-most point
        writer.writerow([f"{-radius:.5f}", "0.00000", alignment_id, "ALIGNMENT_LINE", "LINE"])  # Left-most point
        

def test_and_plot(r=1):
    # Generate coordinates
    coordinates = generate_L_shape_coordinates(radius=r)

    # Extract x and y for plotting
    x_coords = [coord[0] for coord in coordinates]
    y_coords = [coord[1] for coord in coordinates]

    # Plot
    plt.figure(figsize=(10, 10))
    plt.fill(x_coords, y_coords, 'b-', alpha=0.6)  # Fill the L-shape with color for better visualization
    plt.plot(x_coords, y_coords, 'ro-', markersize=5)  # Added 'ro-' to plot points as red dots
    
    # Annotate each point with its ID
    id_value = 0
    for i, (x, y) in enumerate(coordinates):
        plt.annotate(f"{id_value + i}", (x, y), fontsize=8, ha='right')  # Annotate the ID, adjusted placement slightly for readability
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('L Shape with Point IDs')
    plt.axis('equal')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    radius = float(input("Enter the radius of the circle: "))
    filename = "VINYL L_SHAPE_1_pt.csv" 
    test_and_plot(r=radius)
    
    save_to_csv(radius, filename=filename)
    print("Coordinates saved to: " + filename)


