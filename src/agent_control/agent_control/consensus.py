#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime

import pdb

class Consensus(Agent):
    def __init__(self, my_number, my_neighbors=[], *args, sim=False, sync_move=False,
        destination_tolerance=0.01,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
        neighbor_avoid=True, neighbor_delay=5):
        super().__init__(my_number, my_neighbors, sim=sim, sync_move=sync_move, 
                        destination_tolerance=destination_tolerance,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)

    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our concensus logic here.

        Equation:
        new_position = sum of each neighbor [ weight * (ni_pos - my_pos) * (ni_pos - my_pos - 0.5)]

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''

        total = 0
        not_too_close = 0.5

        for name, neighbor in self.neighbor_position.items():
            difference = np.array(neighbor) - np.array(self.position)
            if np.linalg.norm(difference) > not_too_close:
                weight = 1
            else:
                weight = 0
            total += weight * difference

        self.move_direction(total)


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=4
    ## ros2 launch agent_control consensus_batch.launch.py yaml_load:=False number_robots=4
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    rclpy.init(args=args)
    my_robot = Consensus(int(script_args.index), np.array(script_args.neighbor), sim=script_args.sim, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
To Do:
Do sync move? - in agent file
Make a concenses example
'''