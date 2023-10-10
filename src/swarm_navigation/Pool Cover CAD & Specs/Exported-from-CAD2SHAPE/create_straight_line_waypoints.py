import csv

def interpolate_waypoints(start, end, num_points):
    """
    Interpolates between two waypoints linearly.

    Parameters:
    - start: A tuple (x, y, theta) representing the starting waypoint.
    - end: A tuple (x, y, theta) representing the ending waypoint.
    - num_points: The number of total points, including start and end.

    Returns:
    - A list of interpolated waypoints.
    """
    # Unpack waypoints
    x1, y1, theta1 = start
    x2, y2, theta2 = end

    # Calculate intervals
    dx = (x2 - x1) / (num_points - 1)
    dy = (y2 - y1) / (num_points - 1)
    dtheta = (theta2 - theta1) / (num_points - 1)

    # Generate waypoints
    waypoints = [(x1 + i*dx, y1 + i*dy, theta1 + i*dtheta) for i in range(num_points)]
    return waypoints

def write_to_csv(waypoints, filename="waypoints.csv"):
    """
    Writes waypoints to a CSV file.

    Parameters:
    - waypoints: A list of tuples (x, y, theta).
    - filename: Name of the CSV file.
    """
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['x', 'y', 'theta']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for waypoint in waypoints:
            x, y, theta = waypoint
            writer.writerow({'x': x, 'y': y, 'theta': theta})

if __name__ == '__main__':
    start_point = (-1.8, 0.9, 0.0)
    end_point = (1.8, 0.9, 0.0)
    num_points = 100

    waypoints = interpolate_waypoints(start_point, end_point, num_points)
    write_to_csv(waypoints, filename="MESH CIRCULAR_1_pt_straight_line.csv")
