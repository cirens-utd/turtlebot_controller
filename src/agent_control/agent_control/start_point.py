#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from agent_control.agent import Agent
import argparse
import datetime
import yaml
import traceback
from irobot_create_msgs.msg import LightringLeds

import pdb

class StartPoint(Agent):
    def __init__(self, node_name):

        self._extra_param_update_map = {
            "Start.positions": "start_positions",
            "Start.wait": "wait"
        }
        super().__init__(node_name)

        self.declare_parameter("Start.positions", [0.0, 0.0])    # (X1, Y1, X2, Y2, ..., Xn, Yn)
        self.start_positions = self.get_parameter("Start.positions").value if self.get_parameter_or('Start.positions', None).type_ != Parameter.Type.NOT_SET else []

        self.declare_parameter("Start.wait", False)    # Overrides neighbor_walk_around
        self.wait = self.get_parameter("Start.wait").value

        if len(self.start_positions):
            start_point = None
            num_points = len(self.start_positions) // 2

            for n_idx, item in enumerate(self._my_neighbors):
                if self.my_number == item:
                    idx = n_idx % num_points
                    x = self.start_positions[idx * 2]
                    y = self.start_positions[idx * 2 + 1]
                    start_point = (x, y)

            if type(start_point) == type(None):
                raise ValueError(f"Could not find a value for {self.my_number}")
        else:
            self.get_logger().warning(f"{self.my_name}:No Start Positions Passed!!")
            start_point = (0,0)
            

        self.get_logger().info(f"Going to {start_point}")
        self.starting_point = start_point
        # set to true so we don't need to wait on neigbors to move
        self.robot_moving = True
        self.neighbor_walk_around = not self.wait

    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our formation controle logic here.

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''

        # Move to desired start point
        
        self.move_to_position(self.starting_point)

        if self.motion_complete:
            self.robot_status = "FINISHED"
            self.shutdown()
            rclpy.shutdown()

def main(args=None):
    '''
    Pass in all the neighbors and order of their points.
    Script will find which point belongs to this index and move to that point
    '''
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=4

    my_robot = None
    try:
        rclpy.init(args=args)
        my_robot = StartPoint("StartPoint")
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
