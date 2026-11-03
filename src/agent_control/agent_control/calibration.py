#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from agent_control.agent import Agent
import argparse
import yaml
import os
import datetime

import pdb

class Calibration(Agent):
    def __init__(self, node_name, run_setup=True):
        super().__init__(node_name)

        self._use_config_setup = not run_setup

        self.save_points = False
        self.data_pull_file = "data_pull.yaml"

        self.get_logger().info(f"{self.my_name}: Running calibrations")
        self.config_file = "turtlebot_global_config.yaml"
        self._start_heading = 0

        self.x_val_trace = []
        self.y_val_trace = []

        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

        self.calibration_step = 0
        self.rotation_count = 0
        self.num_rotations = 2
        self.half_rotated = False
        self.heading_test_started = False
        self.start_position = None
        self.final_position = None
        self.start_position_heading = None
        self.start_heading = None
        self.final_position_heading = None
        self.final_heading = None
        self.rot_offset_needed = None
        self.heading_timer = None
        self.settle_timer = None

    def controller(self):
        # '''
        # Running calibarations on the robot
        # '''

        if self.calibration_step == 0:
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
                self.final_position = self.pose
                self.calibration_step = self.calibration_step + 1
            else:
                self.move_robot_(0.0, 1.0)

        elif self.calibration_step == 1:
            if not self.heading_test_started or not self.desired_heading:
                self.turn_to_position([0,0])
                self.heading_test_started = True
            else:
                self.calibration_step = self.calibration_step + 1
        elif self.calibration_step == 2:
            if self.start_position_heading is None:
                self.start_position_heading = self.position
                self.start_heading = self.direction_heading
                self.heading_timer = datetime.datetime.now()

            if self.heading_timer + datetime.timedelta(seconds=3) >= datetime.datetime.now():
                self.move_robot_(1.0, 0.0)
            else:
                self.move_robot_(0.0, 0.0)
                if self.settle_timer is None:
                    self.settle_timer = datetime.datetime.now()
                elif self.settle_timer + datetime.timedelta(seconds=1) <= datetime.datetime.now() and self.final_position_heading is None:
                    self.final_position_heading = self.position
                    self.final_heading = self.direction_heading

                    # Actual direction of travel
                    true_heading = np.arctan2(
                        self.final_position_heading[1] - self.start_position_heading[1], 
                        self.final_position_heading[0] - self.start_position_heading[0]) + np.pi

                    # Calibration offset
                    self.rot_offset_needed = true_heading - self.start_heading
                    self.rot_offset_needed = (self.rot_offset_needed + np.pi) % (2 * np.pi) - np.pi 

                    self.publish_results()

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
        shift_x, shift_y = self.compute_offset(self.final_position, [x,y])

        results = {
            "shift_x": float(shift_x),
            "shift_y": float(shift_y),
            "x_diff": float(self.x_max - self.x_min),
            "y_diff": float(self.y_max - self.y_min),
            "heading_offset": float(self.rot_offset_needed)
        }

        if not self._use_config_setup:
            with open(path_to_file, 'w') as file:
                yaml.dump(results, file, default_flow_style=False)

            self.get_logger().info(f"File saved to {path_to_file}")
        else:
            self.get_logger().info(f"Radius: {r}")
            self.get_logger().info(f"X Deviation: {self.x_max - self.x_min}")
            self.get_logger().info(f"Y Deviation: {self.y_max - self.y_min}")
            self.get_logger().info(f"Heading Deviation: {self.rot_offset_needed}")
            self.get_logger().info(f"Using Offsets: {self._offset_x, self._offset_y}")
            self.get_logger().info(f"Using Heading Offsets: {self._offset_heading}")

        if self.save_points:
            results = {
                "shift_x": float(shift_x),
                "shift_y": float(shift_y),
                "x_diff": float(self.x_max - self.x_min),
                "y_diff": float(self.y_max - self.y_min),
                "heading_offset": float(self.rot_offset_needed)
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
    ## ros2 run agent_control calibration.py -c --ros-args -p mode.sim:=true -p laser.avoid:=false -p neighbor.avoid:=false
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--calibrate", default=True, action="store_false", help="Set if you do not want to run calibration")
    script_args, ros_args = parser.parse_known_args()

    my_robot = None
    try:
        rclpy.init(args=ros_args)
        my_robot = Calibration(
            "Calibration",
            script_args.calibrate)
        rclpy.spin(my_robot)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

