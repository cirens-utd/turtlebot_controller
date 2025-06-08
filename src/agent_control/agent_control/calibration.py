#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from agent_control.agent import Agent
import argparse
import yaml
import os

import pdb

class Calibration(Agent):
    def __init__(self, my_number, run_setup=True, *args, sim=False, sync_move=False,
        destination_tolerance=0.01, 
        restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2,
        neighbor_avoid=True, neighbor_delay=5):
        super().__init__(my_number, [], sim=sim, sync_move=sync_move, 
                        destination_tolerance=destination_tolerance, logging=False,
                        restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)

        self._use_config_setup = not run_setup

        self.save_points = False
        self.data_pull_file = "data_pull.yaml"

        self.get_logger().info(f"{self.my_name}: Running calibrations")
        self.config_file = "turtlebot_global_config.yaml"
        self.start_heading = 0

        self.x_val_trace = []
        self.y_val_trace = []

        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

        self.rotation_count = 0
        self.num_rotations = 2
        self.half_rotated = False
        self.start_position = None

    def controller(self):
        '''
        Running calibarations on the robot
        '''
        if type(self.start_position) == type(None):
            self.start_position = self.position
            self.x_min = self.position[0]
            self.x_max = self.position[0]
            self.y_min = self.position[1]
            self.y_max = self.position[1]

        if self.x_min > self.position[0]:
            self.x_min = self.position[0]
        elif self.x_max < self.position[0]:
            self.x_max = self.position[0]
        if self.y_min > self.position[1]:
            self.y_min = self.position[1]
        elif self.y_max < self.position[1]:
            self.y_max = self.position[1]

        self.x_val_trace.append(self.position[0])
        self.y_val_trace.append(self.position[1])

        if self.direction_heading > np.pi:
            self.half_rotated = True
        elif self.half_rotated and self.direction_heading < np.pi:
            self.half_rotated = False
            self.rotation_count += 1

        if self.rotation_count >= self.num_rotations:
            self.move_robot_(0.0, 0.0)
            self.publish_results()
        else:
            self.move_robot_(0.0, 1.0)

    def fit_circle(self, x, y):
        x, y = np.array(x).flatten(), np.array(y).flatten()

        A = np.c_[2*x, 2*y, np.ones(len(x))]
        b = x**2 + y**2
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        xc, yc = c[0], c[1]
        r = np.sqrt(c[2] + xc**2 + yc**2)
        return xc, yc, r
    
    def compute_offset(self, original_pose, desired):
        original = [original_pose['pose']['position']['x'], original_pose['pose']['position']['y']]
        junk = PoseStamped()
        junk.pose.orientation.x = original_pose['pose']['orientation']['x']
        junk.pose.orientation.y = original_pose['pose']['orientation']['y']
        junk.pose.orientation.z = original_pose['pose']['orientation']['z']
        junk.pose.orientation.w = original_pose['pose']['orientation']['w']
        quaternion = junk.pose.orientation

        R = self.quaternion_to_rotation_matrix(quaternion)
        delta = np.array(original) - np.array(desired)
        R_inv = np.linalg.inv(R)
        offset =  R_inv @ np.append(delta, 0)
        return offset[:2]

    def publish_results(self):
        path_to_file = os.path.abspath(os.path.join(os.getcwd(), "Config", self.config_file))
        os.makedirs(os.path.dirname(path_to_file), exist_ok= True)

        x, y, r = self.fit_circle(self.x_val_trace, self.y_val_trace)
        shift_x, shift_y = self.compute_offset(self.pose, [x,y])

        results = {
            "shift_x": float(shift_x),
            "shift_y": float(shift_y),
            "x_diff": float(self.x_max - self.x_min),
            "y_diff": float(self.y_max - self.y_min)
        }

        if not self._use_config_setup:
            with open(path_to_file, 'w') as file:
                yaml.dump(results, file, default_flow_style=False)

            self.get_logger().info(f"File saved to {path_to_file}")
        else:
            self.get_logger().info(f"Radius: {r}")
            self.get_logger().info(f"X Deviation: {self.x_max - self.x_min}")
            self.get_logger().info(f"Y Deviation: {self.y_max - self.y_min}")
            self.get_logger().info(f"Using Offsets: {self._offset_x, self._offset_y}")

        if self.save_points:
            results = {
                "shift_x": float(shift_x),
                "shift_y": float(shift_y),
                "test_x": float(test_x),
                "test_y": float(test_y),
                "x_diff": float(self.x_max - self.x_min),
                "y_diff": float(self.y_max - self.y_min),
                "x_vals": [float(test_x) for test_x in self.x_val_trace],
                "y_vals": [float(test_y) for test_y in self.y_val_trace]
            }
            path_to_file = os.path.abspath(os.path.join(os.getcwd(), "Config", self.data_pull_file))
            os.makedirs(os.path.dirname(path_to_file), exist_ok= True)

            with open(path_to_file, 'w') as file:
                yaml.dump(results, file, default_flow_style=False)

        self.shutdown()
        rclpy.shutdown()





def main(args=None):
    global rclpy
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=1
    ## ros2 run agent_control calibration.py -i 1 -s
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-c", "--calibrate", default=True, action="store_false", help="Set if you do not want to run calibration")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Mode you want to calibrate in")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    rclpy.init(args=args)
    my_robot = Calibration(
        int(script_args.index), 
        script_args.calibrate,
        sim=script_args.sim,
        laser_avoid=False, neighbor_avoid=False)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

