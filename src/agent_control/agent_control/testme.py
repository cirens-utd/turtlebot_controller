#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import yaml
import traceback
from irobot_create_msgs.msg import LightringLeds

import pdb

class TestMe(Agent):
    def __init__(self, node_name):
        super().__init__(node_name)

        # self.robot_ready = True
        self.angles = [0, self.end_heading]
        self.run_index = 0
        self._log_dict_length = 150
        self.led_override = False
        self.counter = 0
        self.mode = 0
        self.select_mode = ["SHUTDOWN", "STOPPED", "READY", "MOVING", "AT_GOAL", "COMPLETE", "FINISHED", "BLOCKED"]
        # self.select_mode = [(0, 178, 255), (0, 255, 100), (89, 178, 255)]
        self.first_run = True

    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our formation controle logic here.

        Equation:
        new_position = sum((np.linalg.norm(neighbor - self.position) - self._formation_distance[i])* neighbor - self.position)
        

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''

        # if self.my_number == 1:
        #     self.move_to_position([2,0])
        # elif self.my_number == 2:
        #     self.move_to_position([-5,5])
        # self.move_to_angle(self.angles[self.test_index])
        # if self.desired_heading:
        #     self.get_logger().info(f"Finished index: {self.test_index}")
        #     self.test_index += 1
        #     if self.test_index > 1:
        #         self.test_index = 0

        # self.move_to_angle(np.pi)
        # pdb.set_trace()

        # self.set_led_mode_(self.select_mode[self.mode])
        # # self.set_led_ring_color(*self.select_mode[self.mode])
        # self.counter += 1
        # if self.counter >= 30:
        #     print(f"LED Color: {self._led_enum[self.select_mode[self.mode]].value}")
        #     self.robot_status = self.select_mode[self.mode]
        #     self.counter = 0
        #     self.mode += 1
        #     if self.mode >= len(self.select_mode):
        #         self.mode = 0
        
        self.move_to_position([3, 2])
        # if "pose" in self.pose:
        #     orientation = self.pose['pose']['orientation']
        #     x = orientation['x']
        #     y = orientation['y']
        #     z = orientation['z']
        #     w = orientation['w']
        #     angle = np.remainder((np.arctan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z)) + np.pi) , 2 * np.pi)
        #     self.get_logger().info(f"Variables: {x}, {y}, {z}, {w}, {x*x + y*y + z*z + w*w}")
        #     self.get_logger().info(f"Angle: {angle}")

        # self.get_logger().info(f"My Heading is: {self.direction_heading}")
        


    def end_controller(self):
        # want to copy the followers angle
        # use self.neighbor_orientation[name]

        desired = self.direction_heading
        num_neighbors = 1
        for name, neighbor in self.neighbor_orientation.items():
            num_neighbors += 1
            desired += self.get_angle_quad(neighbor)

        desired /= num_neighbors

        self.move_to_angle(desired)

def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=1
    ## ros2 run agent_control testme.py -i 1 -s
    '''
    You formation yaml should have robot numbers in it and the formation distances.
    You pass in which node is this one through -i and all the others will be neighbors
    '''

    my_robot = None
    try:
        rclpy.init(args=args)
        my_robot = TestMe("TestMe")
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        if my_robot:
            my_robot.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
