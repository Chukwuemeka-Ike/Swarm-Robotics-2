import csv
import math

def generate_circle_coordinates(radius, num_points=100):
    # Generating the coordinates for the circular shape
    angle_increment = 2 * math.pi / num_points
    coordinates = []

    for i in range(num_points):
        angle = i * angle_increment
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        coordinates.append((x, y))
    
    return coordinates

def save_to_csv(radius, filename="MESH CIRCULAR_1_pt.csv"):
    coordinates = generate_circle_coordinates(radius)
    
    # Writing the coordinates to csv
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["_X", "_Y", "ID", "LAYER", "ENTITY"])

        id_value = 124  # Starting ID value for COVER_HEM
        for x, y in coordinates:
            writer.writerow([f"{x:.5f}", f"{y:.5f}", id_value, "COVER_HEM", "LWPOLYLINE"])

        # Writing the alignment line
        alignment_id = 119
        writer.writerow([f"{-radius:.5f}", "0.00000", alignment_id, "ALIGNMENT_LINE", "LINE"])  # Left-most point
        writer.writerow([f"{radius:.5f}", "0.00000", alignment_id, "ALIGNMENT_LINE", "LINE"])  # Right-most point

if __name__ == "__main__":
    radius = float(input("Enter the radius of the circle: "))
    save_to_csv(radius)
    print(f"Coordinates saved to circle_coordinates.csv")
