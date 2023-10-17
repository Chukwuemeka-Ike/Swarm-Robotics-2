#!/usr/bin/env python3

"""
Author: Burak Aksoy

Description:
    Read a csv file exported from a .DWG CAD file using CAD2SHAPE software.
    The .DWG CAD files includes 2D drawings of a custom fabric shape for a product used in Anchor industries.


"""

# import rospy

# import tf_conversions # For transformations
# import tf2_ros

# import geometry_msgs.msg
# import sensor_msgs.msg

import os
import time

import pandas as pd
import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

from scipy.interpolate import interp1d
from scipy.interpolate import make_interp_spline

import math

import shapely
import shapely.geometry
import shapely.affinity

import meshpy.triangle

import matplotlib.pyplot as plt # To visualize polygons
from matplotlib.animation import FFMpegWriter
import matplotlib.animation as animation
import matplotlib

# Set the default DPI for all images
plt.rcParams['figure.dpi'] = 100  # e.g. 300 dpi
# Set the default figure size
plt.rcParams['figure.figsize'] = [12.8, 9.6]  # e.g. 6x4 inches
# plt.rcParams['figure.figsize'] = [6.4, 4.8]  # e.g. 6x4 inches


class ProcessCustomFabric():
    def __init__(self):
        ################## USER PARAMETERS: BEGIN ###################
        # Get the fabric csv file exported from CAD2Shape.
        # self.path_to_fabric_csv = "./VINYL 716262_1_pt.csv"
        # self.path_to_fabric_csv = "./VINYL 717405_1_pt.csv"
        # self.path_to_fabric_csv = "./MESH 717167_1_pt.csv"
        # self.path_to_fabric_csv = "./MESH 717651_1_pt.csv"
        # self.path_to_fabric_csv = "./MESH CIRCULAR_1_pt.csv"
        self.path_to_fabric_csv = "./VINYL L_SHAPE_1_pt.csv"

        # Get desired AB distance from the user for our custom scale 
        self.desired_fabric_AB_dist = 5.4864 # in meters # if set to 0 or less, original size of the fabric in the csv files will be used.
        
        # Get workstation operator depth (defines the amount of fabric draping outside robots)
        self.ws_op_depth = 0.2 # meters

        # Get workstation width (used to calculate the needed simplifications on the shape)
        self.ws_width = 0.5 # meter
        
        # Radius of an operator seat.
        self.seat_radius = 0.3125 # meter

        # Offset such that the robot path is at least this much inside the fabric edge
        self.user_offset = 0.05 # meter

        # # Get robot attachment surface radius (can be defined with the saucer dome radius )
        # self.attach_r = 0.34 # meters        

        self.save_mode = False

        ################## USER PARAMETERS: END ###################

        # read the fabric csv file
        self.path_to_fabric_csv_os = os.path.expanduser(self.path_to_fabric_csv)
        self.df = pd.read_csv(self.path_to_fabric_csv_os)

        # read the COVER_HEM points which defines the overall fabric shape
        self.df_fabric =  self.df[self.df["LAYER"]=="COVER_HEM"]
        
        # drop duplicate coordinates
        self.df_fabric = self.df_fabric.drop_duplicates(subset=['_X','_Y'], keep="first")
        
        # Get the fabric x y coordinates as a list
        self.fabric_coords = self.df_fabric[['_X','_Y']].values.tolist()

        # Sort the polygon coordinates for counter clockwise order
        # self.fabric_coords = self.sort_polygon_points(self.fabric_coords)
        # NOTE: May not work always! Check results by plotting the coordinates in order as done below.
        
        # print(np.array(self.fabric_coords))
        # print(self.fabric_coords)

        # Create shapely polygon from the coordinates
        r = shapely.geometry.LinearRing(self.fabric_coords)
        r = shapely.geometry.Polygon(r)

        # self.PlotPolygonsWithMatplotlib([r])

        # Get A and B point coordinates that defines the scale and alignment
        fabric_AB = self.df.loc[(self.df["LAYER"]=="ALIGNMENT_LINE") & 
                                (self.df["ENTITY"]=="LINE")][['_X','_Y']].values # in inches
        fabric_AB_vect = fabric_AB[0] - fabric_AB[1]
        fabric_AB_dist = np.linalg.norm(fabric_AB_vect) # in inches
        print("Original AB distance: ", fabric_AB_dist, " inches = ", 
              fabric_AB_dist/(39.3701)," meters." )

        # find the angle of the AB vector
        angle = math.atan2(fabric_AB_vect[1], fabric_AB_vect[0])
        print("ANGLE of AB wrt X-axis before rotation:",np.rad2deg(angle)," deg.")
        
        # Rotate the polygon so that its alignment line aligns with the x axis
        r = shapely.affinity.rotate(r, -angle, origin='centroid', use_radians=True)

        # Translate the polygon so that its centroid is at the origin
        r = shapely.affinity.translate(r, xoff=-r.centroid.x, yoff=-r.centroid.y)

        if self.desired_fabric_AB_dist <= 0.0:
            # or use the default distance to not to scale
            self.desired_fabric_AB_dist = fabric_AB_dist/(39.3701)
            
        print("Desired fabric AB distance = ", self.desired_fabric_AB_dist, " meters."  )
        
        # Calculate the needed scale to achieve the desired size.
        scale_fact = self.desired_fabric_AB_dist / fabric_AB_dist
        print("Scale that will be applied to the fabric in inches to match with the desired size in meters = ", scale_fact)

        # Scale(by scale_fact=xfact=yfact zfact=1.)
        r = shapely.affinity.scale(r, xfact=scale_fact, yfact=scale_fact, origin='centroid') 
        # # The point of origin can be a keyword:
        # 'center' for the 2D bounding box center (default), 
        # 'centroid' for the geometry’s 2D centroid, 
        # a Point object or a coordinate tuple (x0, y0, z0)

        self.fabric_polygon = r # shapely polygon object
        self.fabric_coords = list(self.fabric_polygon.exterior.coords)[:-1]

        # print(np.array(self.fabric_coords))
        
        # # Publish the original polygon to RVIZ
        # self.publish_polygon_RVIZ(self.fabric_coords)

        # Shrink and interpolate the original polygon to be at least within the desired offset with equally distant vertex points.
        self.fabric_polygon_offset, self.robot_path_polygon, operation_offset, min_distance_btw = self.shrink_n_interpolate_polygon(self.fabric_polygon, self.user_offset)
        print("Needed offset to make the robot path polygon contained by the original polygon = "+str(operation_offset)+" m.")
        print("Minimim distance from the robot path polygon to the original polygon = "+str(min_distance_btw) 
              +" m, while user specified: "+str(self.user_offset)+" m.")


        # Show the polygons with matplotlib
        self.PlotPolygonsWithMatplotlib([self.fabric_polygon,
                                         self.fabric_polygon_offset, 
                                         self.robot_path_polygon],save=self.save_mode)
        
        # # Show the polygons with matplotlib
        # self.PlotPolygonsWithMatplotlib([ self.robot_path_polygon],save=self.save_mode)

        simplify_robot_path = True # MAKE THIS DEFAULT FALSE!!
        if simplify_robot_path:
            # Create an equally spaced vertices version of the original polygon for mesh creation
            d = 0.05 # meter, space between vertices
            self.robot_path_polygon = self.equal_distance_vertices_polygon(self.robot_path_polygon, n_waypoints=1/d, k=1)

        
        # Calculate desired positions and orientations for the centroid of the polygon (centroid path)
        self.path_positions, self.path_orientations = self.calculate_centroid_path(self.robot_path_polygon, use_radians=False)

        # print(np.array(self.path_orientations).reshape(np.array(self.path_orientations).shape[0],1))

        # Save the path to a csv file
        filename = self.path_to_fabric_csv + "_path.csv"
        self.save_path_to_csv(self.path_positions,self.path_orientations,filename, use_radians=False)

        # Show the centroid path with matplotlib
        self.PlotCentroidPath(self.path_positions,self.path_orientations, use_radians=False, save=self.save_mode)


        # Interpolate for a equal distance waypoints of the centroid
        self.path_positions2, self.path_orientations2 = self.equal_distance_waypoints(self.path_positions,self.path_orientations, n_waypoints=20, use_radians=False)

        # Save the path to a csv file
        filename2 = self.path_to_fabric_csv + "_path_equal_centroid.csv"
        self.save_path_to_csv(self.path_positions2, self.path_orientations2, filename2, use_radians=False)

        # Show the equal distance centroid path with matplotlib
        self.PlotCentroidPath(self.path_positions2, self.path_orientations2, use_radians=False, save=self.save_mode, save_file_addition="_equal_centroid_d")


        # Animate fabric motions
        self.AnimateMotionWithMatplotlib(self.fabric_polygon, 
                                         self.robot_path_polygon, 
                                         self.path_positions,  
                                         self.path_orientations,  
                                         interval_ms=25, 
                                         use_radians=False, 
                                         save=self.save_mode)
        
        self.AnimateMotionWithMatplotlib(self.fabric_polygon,
                                         self.robot_path_polygon, 
                                         self.path_positions2, 
                                         self.path_orientations2, 
                                         interval_ms=25, 
                                         use_radians=False, 
                                         save=self.save_mode, 
                                         save_file_addition="_equal_centroid_d")
        

        # --- Create mesh with triangles from the original fabric ----
        # Create an equally spaced vertices version of the original polygon for mesh creation
        d = 0.2 # meter, space between vertices
        self.fabric_polygon_eq_dist = self.equal_distance_vertices_polygon(self.fabric_polygon, n_waypoints=1/d, k=1)
        
        # Show the difference between the original polygon and the equally spaced vertices polygons with matplotlib
        self.PlotPolygonsWithMatplotlib([self.fabric_polygon,
                                         self.fabric_polygon_eq_dist],save=False)

        # Create the mesh
        self.mesh = self.create_mesh(self.fabric_polygon_eq_dist, max_area=d**2/2.0)

        # Show the created mesh
        self.PlotMesh(self.mesh, save=False)

        # Export the created mesh as an obj file
        if self.save_mode:
            self.export_mesh_as_obj(self.mesh, self.path_to_fabric_csv+".obj")

    def save_path_to_csv(self, path_positions, path_orientations, filename, use_radians=True):
        # This function will always exports the orientations as radians
        # hence, if use_radians false, we assume that the orientations are given in degrees 
        # and we will convert theme to radians before saving.

        # Check if input lists have the same length
        if len(path_positions) != len(path_orientations):
            raise ValueError('Both lists must have the same length.')
        
        # Convert to degrees if use_radians is False
        if not use_radians:
            path_orientations = [self.wrapToPi(np.deg2rad(orient)) for orient in path_orientations]
        else:
            path_orientations = [self.wrapToPi(orient) for orient in path_orientations]

        # Create a DataFrame from the lists
        df = pd.DataFrame({
            'x': [pos[0] for pos in path_positions],
            'y': [pos[1] for pos in path_positions],
            'theta': path_orientations
        })

        # Save the DataFrame to a CSV file
        df.to_csv(filename, index=False)

    def export_mesh_as_obj(self, mesh, filename):
        with open(filename, 'w') as f:
            for p in mesh.points:
                f.write("v {} {} {}\n".format(p[0], p[1], 0)) # z-coordinate is 0 as it is a 2D mesh
            for element in mesh.elements:
                f.write("f")
                for vertex in element:
                    f.write(" {}".format(vertex + 1)) # .obj format uses 1-indexing
                f.write("\n")
    
    def create_mesh(self, polygon, max_area=0.01):
        poly_points = list(polygon.exterior.coords)[:-1]

        def round_trip_connect(start, end):
            """
            creates a set of pairs where each pair contains an index 
            and the index of the next point, it helps to define the 
            edges of the polygon.
            """
            return [(i, i+1) for i in range(start, end)] + [(end, start)]
    
        info = meshpy.triangle.MeshInfo()
        info.set_points(poly_points)
        info.set_facets(round_trip_connect(0, len(poly_points)-1))

        mesh = meshpy.triangle.build(info, max_volume=max_area)

        # mesh_points = np.array(mesh.points) # N by 2 [x y] points array, N: Number of vertices
        # mesh_tris = np.array(mesh.elements) # M by 3 [v1 v2 v3] vertex indices array that forms the triangles, M: Number of triangles
        # mesh_facets = np.array(mesh.facets) # K by 2 [v1 v2] vertex indices array that defines the boundary edges of the mesh, K: Number of edges, (not necessarily ordered)

        return mesh

    def equal_distance_vertices_polygon(self, polygon, n_waypoints=100, k=3):
        """
        Interpolates the vertices of a polygon such that they have equal distances between each other
        - points: positions of vertices as a list of lists [[x0,y0],[x1,y1],...] 
        - n_waypoints: number of waypoints per meter!
        - k: B-spline degree. Default is cubic, k = 3.

        Note that polygon is supposed to be a closed loop. The function enforces continuity between the end and start of the polygon
        by treating the polygon as periodic and use a periodic spline interpolation function that handles the end condition 
        by wrapping around to the other side of the function, thereby creating a seamless loop by
        Using scipy's make_interp_spline function with a 'periodic' boundary condition.

        Periodic boundary condition means the values and the first k-1 (for cubic k=3) derivatives at the ends are equivalent.

        Note: if the polygon isn't strictly convex, or if it has a complex shape, some of the interpolated points might lie outside the original polygon.
        """
        # Get the x and y coordinates of the polygon's exterior
        x, y = polygon.exterior.xy

        # creating the cumulative distance array, setting the start point as 0
        cumulative_distance = np.zeros(len(x))
        for i in range(1, len(x)):
            # calculating euclidean distance
            cumulative_distance[i] = cumulative_distance[i-1] + np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)

        # defining the interpolation function for x, y coordinates
        spl = make_interp_spline(cumulative_distance, np.c_[x, y], k=k, bc_type='periodic')

        # defining new waypoints, spaced evenly along the total path length
        perimeter = cumulative_distance[-1]
        n_pts = math.ceil(perimeter*n_waypoints)

        new_distances = np.linspace(0, perimeter, n_pts)

        # getting the new waypoints
        new_points = spl(new_distances)

        # create the new polygon with the new vertex points
        r = shapely.geometry.LinearRing(new_points[:-1])
        r = shapely.geometry.Polygon(r)
        
        return r
    
    def equal_distance_waypoints(self, points, headings, n_waypoints=100, use_radians=True):
        """
        - points: positions of waypoints as a list of lists [[x0,y0],[x1,y1],...] 
        - headings: orientation angles of waypoint frames as a list [th0,th1,...]
        - n_waypoints: number of waypoints per meter!
        """
        # Then we will interpolate
        x = [pt[0] for pt in points]
        y = [pt[1] for pt in points]
        angles = []
        if use_radians:
            angles = headings
        else:
            angles = [np.deg2rad(angle) for angle in headings]
        
        # unwrapping the angles
        unwrapped_angles = np.unwrap(angles)

        # creating the cumulative distance array, setting the start point as 0
        cumulative_distance = np.zeros(len(points))
        for i in range(1, len(points)):
            # calculating euclidean distance
            cumulative_distance[i] = cumulative_distance[i-1] + np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)

        # defining the interpolation function for x, y coordinates
        f_xy = interp1d(cumulative_distance, np.c_[x, y], kind='cubic', axis=0)

        # defining the interpolation function for angles
        f_angles = interp1d(cumulative_distance, unwrapped_angles, kind='linear')

        # defining new waypoints, spaced evenly along the total path length
        perimeter = cumulative_distance[-1]
        n_pts = math.ceil(perimeter*n_waypoints)

        new_distances = np.linspace(0, perimeter, n_pts)

        # getting the new waypoints
        new_points = f_xy(new_distances)
        new_headings = f_angles(new_distances)

        # wrapping the angles back to [-pi, pi)
        if use_radians:
            new_headings = np.mod(new_headings + np.pi, 2 * np.pi) - np.pi
        else:
            new_headings = np.rad2deg(np.mod(new_headings + np.pi, 2 * np.pi) - np.pi)

        return new_points, new_headings

    def calculate_centroid_path(self, polygon, use_radians=True):
        """
        Calculates the required path positions and orientations of a polygon centroid to make the world origin frame tracks the edges of the polygon. 

        Returns: 
        - positions
        - orientations

        Notes: 
        - This function assumes that the polygon parameter is already translated to be around its centroid (i.e this function does not re-calculate the centroid).
        Also make sure the given polygon is oriented to be counter clockwise.
        """
    
        # Extract exterior coordinates
        ext_coords = list(polygon.exterior.coords)[:-1]
        # print(shapely.LinearRing(ext_coords).is_ccw)

        positions = []
        orientations = []

        # Loop over edges of the polygon
        for i in range(len(ext_coords)):
            if i == (len(ext_coords) - 1): # end loop condition
                edge = shapely.geometry.LineString([ext_coords[i], ext_coords[0]])
                # Compute delta x and delta y
                dx = ext_coords[0][0] - ext_coords[i][0]
                dy = ext_coords[0][1] - ext_coords[i][1]
            else:
                # Define edge as a LineString
                edge = shapely.geometry.LineString([ext_coords[i], ext_coords[i+1]])

                # Compute delta x and delta y
                dx = ext_coords[i+1][0] - ext_coords[i][0]
                dy = ext_coords[i+1][1] - ext_coords[i][1]

            # Compute angle with respect to x-axis
            angle = np.rad2deg(np.arctan2(dy, dx))
            
            tol = 1e-03
            if np.linalg.norm(np.array([dx,dy])) <= tol:
                # ignore this point because of sensitivity in the arctan
                continue
                # print("dx: ", dx, "dy: ", dy )
                # print("i: ", len(ext_coords)-i-1, "angle: ", angle)

            # Add position and orientation to the lists
            
            # Compute centroid of the edge in polygon frame
            edge_centroid = np.array([edge.centroid.x,edge.centroid.y,1.0])
            # Compute the polygon centroid coordinate from centroid of the edge
            poly_centroid = (self.rot_mat(np.deg2rad(-angle)).dot(-edge_centroid))[0:2]
            
            positions.append(poly_centroid.tolist())
            if use_radians:
                orientations.append(np.deg2rad(-angle))  # Rotate by negative angle to align with x-axis
            else:
                orientations.append(-angle)  # Rotate by negative angle to align with x-axis


        return positions[::-1], orientations[::-1]
            
    def curvature_radius(self, r,w,d):
        """
        Rule of thumb curvature calculation function to smooth out the fabric polygon.
        - r: radius of the operator seat that is an half circle sticked to the workstation rectangle
        - w: width of the workstation rectangle
        - d: distance of the operation point towards the inside of the workstation from the width defining edge. 
        """
        # return math.sqrt(d**2 + (w/2.0)**2)
        return math.sqrt((r + w/2)**2 + (r-d)**2)

    def shrink_polygon(self,polygon,offset):
        """
        Shrink a polygon by a specified positive offset
        """
        polygon_shrink = polygon
        polygon_shrink = polygon.buffer(-1.0*offset, cap_style="round", join_style="round") # erode
        return polygon_shrink

    def smooth_polygon(self,polygon,radius):
        """
        - polygon: Shapely polygon object
        - radius: desired radius to smoot sharper curves (must be a positive number)
        
        To achieve that, we will dilate x, erode by 2x and then dilate again by x. 
        (see: https://colab.research.google.com/drive/1kZGh7n7SYgX1VSeveENFu_Tg38aBPlvT?usp=sharing)
        based on a question asked in here: # https://gis.stackexchange.com/a/93235/226427
        """
        polygon_smooth = polygon
        polygon_smooth = polygon_smooth.buffer(1.0*radius, cap_style="round", join_style="round") # dilate by x
        polygon_smooth = polygon_smooth.buffer(-2.0*radius, cap_style="round", join_style="round") # erode by 2x
        polygon_smooth = polygon_smooth.buffer(1.0*radius, cap_style="round", join_style="round") # dilate by x
        
        # Make sure the smoothed polygon is properly oriented
        polygon_smooth = shapely.geometry.polygon.orient(polygon_smooth, sign=1.0)
        # print(polygon_smooth.is_valid)
        # print(len(polygon_smooth.geoms))
        return polygon_smooth

    def shrink_n_interpolate_polygon(self, polygon, user_offset):
        """
        Iteratively shrink the original polygon to calculate the robot path polygon
        such that the robot path polygon is completely contained by the original polygon in the end
        Each iteration applies a shrinking operation by 1 cm = 0.01 m.
        Once the robot path polygon is completely contained by the original polygon, we will keep shrinking
        until the user specified minimum offset distance is reached from the edge of original polygon.
        """

        operation_offset = 0.0 # meter
        min_distance_btw = -1 # parameter initialization for min distance between the inner robot path polygon and original polygon.

        # i = 0
        while True:
            # shrink the fabric polygon with some offset 
            offset_polygon = self.shrink_polygon(polygon, operation_offset)

            # offset_polygon = self.smooth_polygon(offset_polygon, 0)
            # Make sure the smoothed polygon is properly oriented
            offset_polygon = shapely.geometry.polygon.orient(offset_polygon, sign=1.0)

            # interpolate the polygon 
            robot_path_polygon = self.equal_distance_vertices_polygon(offset_polygon,n_waypoints=100)

            # check if the robot path polygon is contained by the original polygon
            if (polygon.contains(robot_path_polygon)):
                min_distance_btw = robot_path_polygon.boundary.distance(polygon.boundary)
                if (min_distance_btw >= user_offset):
                    break
                # else:
                    # print("Current min distance: " + str(min_distance_btw))
            # else increase the shrinking amount
            operation_offset += 0.01 # meter
            # i = i + 1
            # print(i)

        return offset_polygon, robot_path_polygon, operation_offset, min_distance_btw

    def PlotMesh(self,mesh, save=False):
        mesh_points = np.array(mesh.points)
        mesh_tris = np.array(mesh.elements)

        # Show the created mesh
        plt.triplot(mesh_points[:, 0], mesh_points[:, 1], mesh_tris)

        # Highlight the first and last points
        plt.scatter(*mesh_points[0], color='red')
        plt.scatter(*mesh_points[-1], color='blue')

        # Annotate each point with its index
        for i, (x, y) in enumerate(mesh_points):
            plt.annotate(str(i), (x, y), fontsize=6)

        # Set the aspect ratio of the axes to be equal
        plt.axis('equal')

        # # Set the axes limits
        # plt.xlim([-4, 4])
        # plt.ylim([-2, 4])

        # Show grid lines
        plt.grid(True)

        if save:
            # Save the figure
            plt.savefig(self.path_to_fabric_csv + "_mesh.png", dpi=300)

        plt.show()    

    def PlotPolygonsWithMatplotlib(self, polygons, save=False):
        # Create a plot
        fig, ax = plt.subplots()

        for polygon in polygons:
            # Get the x and y coordinates of the polygon's exterior
            x, y = polygon.exterior.xy

            # Add the polygon to the plot
            ax.plot(x, y, linewidth=4.0)
            
            # # Plot data points
            # ax.scatter(x, y, color='red')

            # # Annotate data points
            # for i, txt in enumerate(x):
            #     ax.annotate(i, (x[i], y[i]))
            
        # Set aspect ratio to be equal
        ax.set_aspect('equal', adjustable='datalim')
        # Show grid
        ax.grid(True)

        if save:
            # Save the figure
            plt.savefig(self.path_to_fabric_csv + ".png", dpi=300)

        # Show the plot
        plt.show()

    def PlotCentroidPath(self, positions, orientations, use_radians=True, save=False, save_file_addition=""):
        # Split the list of points into two lists of x and y coordinates
        x = [point[0] for point in positions]
        y = [point[1] for point in positions]

        # Create a scatter plot
        plt.scatter(x, y)

        # Draw a line between the points
        plt.plot(x, y, 'g-')  # 'g-' specifies a green line   

        # Set the aspect ratio of the axes to be equal
        plt.axis('equal')

        # # Set the axes limits
        # plt.xlim([-4, 4])
        # plt.ylim([-2, 4])

        # Show grid lines
        plt.grid(True)

        # Annotate points with their index numbers
        for i, point in enumerate(positions):
            plt.annotate(i, point, fontsize=6) 

        # Plot the orientation vectors
        arrow_length = 0.05  # You can set this to whatever you like
        if use_radians:
            dx = [arrow_length * np.cos(orientation) for orientation in orientations]
            dy = [arrow_length * np.sin(orientation) for orientation in orientations]
        else:
            dx = [arrow_length * np.cos(np.deg2rad(orientation)) for orientation in orientations]
            dy = [arrow_length * np.sin(np.deg2rad(orientation)) for orientation in orientations]
        plt.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='r', width=0.001)

        if save:
            # Save the figure
            plt.savefig(self.path_to_fabric_csv + "_centroid"+ save_file_addition +".png", dpi=300)

        # Show the plot
        plt.show()

    def AnimateMotionWithMatplotlib(self, poly, poly2, positions, orientations, interval_ms=500, use_radians=True, save=False, save_file_addition=""):
        fig = plt.figure()
        ax = plt.gca()
        # Set aspect ratio to be equal
        ax.set_aspect('equal', adjustable='datalim')
        # Show grid
        ax.grid(True)

        # Draw x and y axes
        ax.axhline(0, color='red',linewidth=2.5)
        ax.axvline(0, color='green',linewidth=2.5)
        
        # Initialize patch
        patch = matplotlib.patches.Polygon(list(poly.exterior.coords), edgecolor='black', linewidth=2.5)
        ax.add_patch(patch)

        patch2 = matplotlib.patches.Polygon(list(poly2.exterior.coords), edgecolor='yellow', linewidth=2.0)
        ax.add_patch(patch2)

        # Set plot limits to include all movements
        ax.set_xlim(-4, 4)
        ax.set_ylim(-2, 4)

        # Initialize frame info text
        text = ax.text(0, 0, '', fontsize=20)

        # Initialize quiver for robot's local x (heading) and y axes
        arrow_length = 0.5  # meter
        Q_x = ax.quiver([1], [1], [1], [1], angles='xy', scale_units='xy', scale=1, color='r', zorder=3, width=0.005)
        Q_y = ax.quiver([1], [1], [-1], [1], angles='xy', scale_units='xy', scale=1, color=(0,1,0,1), zorder=3, width=0.005)

        # Animation update function
        def animate(i):
            temp_poly = poly
            temp_poly = shapely.affinity.rotate(temp_poly, orientations[i], origin=(0,0), use_radians=use_radians)
            temp_poly = shapely.affinity.translate(temp_poly, xoff=positions[i][0], yoff=positions[i][1])
            patch.set_xy(list(temp_poly.exterior.coords))

            temp_poly2 = poly2
            temp_poly2 = shapely.affinity.rotate(temp_poly2, orientations[i], origin=(0,0), use_radians=use_radians)
            temp_poly2 = shapely.affinity.translate(temp_poly2, xoff=positions[i][0], yoff=positions[i][1])
            patch2.set_xy(list(temp_poly2.exterior.coords))

            # Update quiver
            if use_radians:
                dx_x = arrow_length * np.cos(orientations[i])  # x component (cosine) for local x axis (heading)
                dy_x = arrow_length * np.sin(orientations[i])  # y component (sine) for local x axis (heading)
            else:
                dx_x = arrow_length * np.cos(np.deg2rad(orientations[i]))  # x component (cosine) for local x axis (heading)
                dy_x = arrow_length * np.sin(np.deg2rad(orientations[i]))  # y component (sine) for local x axis (heading)
            
            Q_x.set_UVC(dx_x, dy_x)
            Q_x.set_offsets((positions[i][0], positions[i][1]))
            Q_x.set_visible(True)

            if use_radians:
                dx_y = -arrow_length * np.sin(orientations[i])  # x component (-sine) for local y axis
                dy_y = arrow_length * np.cos(orientations[i])  # y component (cosine) for local y axis
            else:
                dx_y = -arrow_length * np.sin(np.deg2rad(orientations[i]))  # x component (-sine) for local y axis
                dy_y = arrow_length * np.cos(np.deg2rad(orientations[i]))  # y component (cosine) for local y axis
            Q_y.set_UVC(dx_y, dy_y)
            Q_y.set_offsets((positions[i][0], positions[i][1]))
            Q_y.set_visible(True)


            # Update text
            text.set_text('Frame: {}/{}'.format(i+1, len(positions)))
            text.set_position((positions[i][0], positions[i][1]))

        # Create animation
        ani = animation.FuncAnimation(fig, animate, frames=len(positions), interval=interval_ms)

        if save:
            # Specify the writer
            writer = FFMpegWriter(fps=25, metadata=dict(artist='Me'), bitrate=1800)
            # Save as mp4
            ani.save(self.path_to_fabric_csv + save_file_addition +".mp4", writer=writer, dpi=300)

        # Show the plot
        plt.show()

    def sort_polygon_points(self, points):
        """
        Sorts the points of a (convex or non-convex) polygon counterclockwise.
        Note: assumes that the points you are sorting form a single, simple polygon (without holes and intersections)
        """
        # Find the center point
        center_x = sum(p[0] for p in points) / len(points)
        center_y = sum(p[1] for p in points) / len(points)
        center = (center_x, center_y)

        # Define a function for calculating the angle between the center and a given point
        def angle(point):
            return math.atan2(point[1] - center[1], point[0] - center[0])

        # Sort the points based on their angle
        return sorted(points, key=angle)

    # def publish_polygon_RVIZ(self,polygon_coords):
    #     """
    #     - polygon_coords: Coordinates of the polyon to be published as a list
    #     - self.tf_world_frame_id: string of the frame name that is polygon is defined in.
    #     - self.pub_viz_workspace_polygon: publisher object for RVIZ
    #     Publishes the polygon to RVIZ
    #     """
    #     poly_msg = geometry_msgs.msg.PolygonStamped()

    #     poly_msg.header.stamp = rospy.Time.now()
    #     poly_msg.header.frame_id = self.tf_world_frame_id

    #     poly_msg.polygon.points = [geometry_msgs.msg.Point32(x=corner[0], y=corner[1]) for corner in polygon_coords] 

    #     self.pub_viz_workspace_polygon.publish(poly_msg)

    def rot_mat(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0], [s, c, 0], [0,0,1]])

    def wrapToPi(self, a):
        '''
        Wraps angle to [-pi,pi)
        '''
        return ((a+np.pi) % (2*np.pi))-np.pi

if __name__ == '__main__':
    processCustomFabric = ProcessCustomFabric()
    # rospy.spin()