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
    def __init__(self, my_number, my_neighbors=[], start_point=[], *args, 
        sim=False, sync_move=False, viewer=False,
        logging=False, restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
        destination_tolerance=0.01, angle_tolerance=0.1,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
        neighbor_avoid=True, neighbor_delay=5):

        super().__init__(my_number, my_neighbors, sync_move=sync_move, sim=sim, viewer=viewer,
                        destination_tolerance=destination_tolerance, logging=logging, angle_tolerance=angle_tolerance,
                        restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)

        self.get_logger().info(f"Going to {start_point}")
        self.starting_point = start_point
        # set to true so we don't need to wait on neigbors to move
        self.robot_moving = True

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
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("-r", "--record", default=False, action="store_true", help="Enable Logging")
    parser.add_argument("-p", "--points", nargs='+', type=float, help="List of coordinates defining the starting points (e.g. x1 y1 x2 y2 ...)")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    start_point = None
    for n_idx, item in enumerate(script_args.neighbor):
        idx = n_idx
        catchMe = 0
        while idx >= len(script_args.points):
            idx -= len(script_args.points)
            catchMe += 1
            if catchMe == 1000:
                raise ValueError(f"Bro, why are you in this while loop for so long?")

        if script_args.index == item:
            x, y = script_args.points[idx*2], script_args.points[idx*2+1]
            start_point = (x, y)

    if type(start_point) == type(None):
        raise ValueError(f"Could not find a value for {script_args.index}")

    try:
        rclpy.init(args=args)
        my_robot = StartPoint(int(script_args.index), np.array(script_args.neighbor), np.array(start_point), sim=script_args.sim, 
            logging=script_args.record, restricted_area=False, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
