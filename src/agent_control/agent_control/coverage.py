#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, LineString, Point, box
import argparse
import datetime
import traceback

import pdb

class Coverage(Agent):
    def __init__(self, my_number, my_neighbors=[], *args, 
        logging=False,
        perimeter=[(-3.8, 0.19), (-0.12, 3.36), (2.69, 2.0), (2.74, -2.6), (-0.18, -5.0), (-2.36, -5.12)],
        sim=False, sync_move=False, destination_tolerance=0.01,
        restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
        neighbor_avoid=True, neighbor_delay=5, viewer=False):
        super().__init__(my_number, my_neighbors, sim=sim, sync_move=sync_move, 
                        destination_tolerance=destination_tolerance, logging=logging,
                        restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay, viewer=viewer)

        self.get_logger().info(f"My Parameter is {perimeter}")
        self.boundary = Polygon(perimeter) # Points for your boundary [(0,0), (0,1), (1,1), (1,0)]
        self.vor = None
        self.centroids = None
        self.region_polys = None
        self.test_num = 0

    # Reconstruct finite Voronoi polygons with clipping
    def bounded_voronoi_region(self, vor, point_idx, bounding_shape, far_enough=100):
        region_idx = vor.point_region[point_idx]
        region = vor.regions[region_idx]
        if -1 in region or len(region) == 0:
            # Region is infinite, reconstruct polygon with ridge directions
            point = vor.points[point_idx]
            ridges = [r for r in vor.ridge_points if point_idx in r]
            lines = []
            for (p1_idx, p2_idx), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
                if point_idx not in (p1_idx, p2_idx):
                    continue
                if v1 == -1 or v2 == -1:
                    # Get the two original points that generated this ridge
                    p1 = vor.points[p1_idx]
                    p2 = vor.points[p2_idx]

                    # Compute midpoint of p1 and p2 (for visualizing or anchoring)
                    midpoint = (p1 + p2) / 2

                    # Compute the vector between them
                    line_vec = p2 - p1

                    # Compute the normal (perpendicular vector)
                    direction = np.array([-line_vec[1], line_vec[0]])
                    normal = direction / np.linalg.norm(direction)  # Normalize

                    # Choose Direction
                    finite_vertex = vor.vertices[v1 if v1 != -1 else v2]
                    to_vertex = finite_vertex - midpoint

                    # If dot product is negative, we are going the wrong way
                    if np.dot(to_vertex, normal) > 0:
                        normal *= -1 
                    
                    far_point = finite_vertex + far_enough * normal
                    line = LineString([finite_vertex, far_point])

                else:
                    line = LineString([vor.vertices[v1], vor.vertices[v2]])
                lines.append(line)
            try:
                region_poly = Polygon(LineString(np.concatenate([l.coords for l in lines])).convex_hull)
            except Exception as e:
                return None
        else:
            region_poly = Polygon([vor.vertices[i] for i in region])
        
        # Clip with boundry
        return region_poly.intersection(bounding_shape)

    def controller(self):
        '''
        First step: Compute Vernoli and then find the center of each region for the bounded system.
        Robots will move to the center
        '''
        my_run = self.test_num
        self.test_num += 1
        # self.get_logger().info(f"Starting Controller {my_run}: {datetime.datetime.now()}")
        
        # Find Voronoi
        points = list(self.neighbor_position.values()) + [self.position]
        self.vor = Voronoi(points)

        # Get centroids for all regions
        self.centroids = None
        self.region_polys = []
    
        # note: could find all the points with this method
        # for i in range(len(points)):

        i = len(points) - 1
        region_poly = self.bounded_voronoi_region(self.vor, i, self.boundary)

        x,y = region_poly.exterior.xy

        if region_poly and not region_poly.is_empty:
            centroid = region_poly.centroid
            if type(self.centroids) == type(None):
                self.centroids = np.array([(centroid.x, centroid.y)])
            else:
                self.centroids = np.vstack([self.centroids, (centroid.x, centroid.y)])
        else:
            self.centroids = np.append(self.centroids, (None))
        self.region_polys.append(region_poly)

        if type(self.centroids) != type(None) and type(self.centroids[0]) != type(None):
            # self.get_logger().info(f"Going to position: {self.centroids[0]}")
            self.move_to_position(self.centroids[0])
        else:
            self.move_to_position(self.position)

        # self.get_logger().info(f"End Controller {my_run}: {datetime.datetime.now()}")
        
def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=3
    ## ros2 run agent_control coverage.py -i 1 -s -n 2 3 -p 0 0 0 2 2 2 2 0

    '''
    New modules needed:
    - scipy
    - shapely
    '''

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("-p", "--perimeter", nargs='+', type=float, help="List of coordinates defining the ploygon (e.g. x1 y1 x2 y2 ...)")
    parser.add_argument("-r", "--record", default=False, action="store_true", help="Enable Logging")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    if script_args.perimeter:
        points = [(script_args.perimeter[i], script_args.perimeter[i+1]) for i in range(0, len(script_args.perimeter), 2)]
    else:
        points = [(-3.8, 0.19), (-0.12, 3.36), (2.69, 2.0), (2.74, -2.6), (-0.18, -5.0), (-2.36, -5.12)]

    try:
        rclpy.init(args=args)
        my_robot = Coverage(int(script_args.index), np.array(script_args.neighbor), perimeter=points,
            sim=script_args.sim, logging=script_args.record,
            restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()