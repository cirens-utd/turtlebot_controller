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

class StartPoint(Agent):
    def __init__(self, node_name, start_points, wait=False):

        super().__init__(node_name)

        start_point = None
        for n_idx, item in enumerate(self._my_neighbors):
            idx = n_idx
            catchMe = 0
            while idx >= len(start_points):
                idx -= len(start_points)
                catchMe += 1
                if catchMe == 1000:
                    raise ValueError(f"Bro, why are you in this while loop for so long?")

            if self.my_number == item:
                x, y = start_points[idx*2], start_points[idx*2+1]
                start_point = (x, y)

        if type(start_point) == type(None):
            raise ValueError(f"Could not find a value for {self.my_number}")
            

        self.get_logger().info(f"Going to {start_point}")
        self.starting_point = start_point
        # set to true so we don't need to wait on neigbors to move
        self.robot_moving = True
        self.neighbor_walk_around = not wait

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
    parser = argparse.ArgumentParser()
    parser.add_argument("-w", "--wait", default=False, action="store_true", help="Set to make robot just wait for neighbors to move")
    parser.add_argument("-s", "--start_points", nargs='+', type=float, help="List of coordinates defining the starting points (e.g. x1 y1 x2 y2 ...)")
    script_args, ros_args = parser.parse_known_args()

    try:
        rclpy.init(args=ros_args)
        my_robot = StartPoint("StartPoint", script_args.start_points, script_args.wait)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
