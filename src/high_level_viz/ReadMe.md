# High Level Planner Visualizer Package
This package is designed to take the robot positions in a csv file with timestamps and sends message to Gazebo to update their locations.

Allows to quickly visualize the solutions of high level planner realistically. 
Reads a .csv file filled with time stamps (t) and robot poses (x,y,𝜭)
Updates robots poses with user specified intervals (e.g. 2 seconds)
Also shows the time stamps with a TEXT_VIEW_FACING visualization marker in RVIZ.

An example csv file
```
t,x,y,th,x,y,th,x,y,th,x,y,th
1,1,1,0,-1,1,0,-1,-1,0,1,-1,0
60,2,1,90,0,1,90,0,-1,90,2,-1,90
350,3,1,0,1,1,0,1,-1,0,3,-1,0
3950,-1,1.5,-90,-2,1.5,-90,-3,1.5,-90,-4,1.5,-90
39500,-0.5,-0.75,180,-1.5,-1.25,180,-2.2,-0.75,-180,-3,-1.25,-180
45000,4,2,270,4,-1,270,1,2,270,1,-1,270
```

# For early progress of the high level planner:

When the high level planner is not yet matured to directly output the corresponding robot poses with time steps,
but able to output only the robot workstation numbers and ids,
we can use a workaround. We can hard code what should be the robot poses when they are assigned to a workstation.

For example consider a csv file as early output of the high level planner:

high_level_output_early.csv:
```csv
t,  rob_0, rob_1, ...
1, ws_1_0,   .  , ...
.,   .   ,   .  , ... 
.,   .   ,   .  , ...
.,   .   ,   .  , ...
```
The row starting with `"1,ws_1_0"` can be read as: `"At time (`t=1`) seconds, The first robot (`rob_0`) is at the first (`0`) workstation 1 (`ws_1`), ..."`)

So we assume:

| Workstation Name| Workstation Number | Quantity | Explanation   |
| ---             | ---                | ---      | ---           |
| Loading Area    | WS0                | 1        | We have 1 WS0 |
| Mega Stitch     | WS1                | 2        | We have 2 WS1 |
| Welder          | WS2                | 1        | We have 1 WS2 |
| Perimeter Job   | WS3                | 3        | We have 3 WS3 |
| Inspection Area | WS4                | 1        | We have 1 WS4 |

|Number of Robots|
|--- |
15

Each workstation has a csv file that hardcodes where a robot should be when it is assigned to that
For example, 

ws_0_0.csv:
```csv
robot_id, x, y, th
       0, 1, 2, 0
       1, 2, 2, 0
```
ws_1_0.csv:
```csv
robot_id, x, y, th
       0, 1, 4, 0
       1, 2, 4, 0
```

Hence we have the following csv files based on the quantity assumptions:
```
ws_0_0.csv
ws_1_0.csv
ws_1_1.csv
ws_2_0.csv
ws_3_0.csv
ws_3_1.csv
ws_3_2.csv
ws_4_0.csv
```
with 15 rows that is corresponding to the assumed 15 robots.

A python file will parse the `high_level_output_early.csv` file, the hardcoded pose information csv files (`ws_0_0.csv, ws_1_0.csv, ...`), and generate the proper pose csv file that this package can use to visualize.
