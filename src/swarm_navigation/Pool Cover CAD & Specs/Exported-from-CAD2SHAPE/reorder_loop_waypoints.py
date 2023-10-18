import csv

# Read the CSV file
filename = "ticket_7.csv"
filename_new = "ticket_7_new.csv"

new_initial_waypoint_index = 202


with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    headers = next(csvreader)  # Get the headers
    waypoints = list(csvreader)  # Convert remaining rows to a list

# Rearrange the waypoints
adjusted_waypoints = waypoints[new_initial_waypoint_index-1:] + waypoints[:new_initial_waypoint_index-1]

# Write back to the same CSV file
with open(filename_new, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(headers)  # Write headers first
    csvwriter.writerows(adjusted_waypoints)  # Write the adjusted waypoints

print("Waypoints have been adjusted.")
