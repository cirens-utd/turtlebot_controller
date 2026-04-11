#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import traceback

import pdb

class Consensus(Agent):
    def __init__(self, node_name):
        super().__init__(node_name)


        #self._neighbor_tolerance
        self.stopping_distance = (self._diameter + self._neighbor_tolerance + 0.1) / np.sin(np.pi / len(self._neighbors_ready))
        self.complete = False
        self.get_logger().info(f"Running most recent changes")

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
        '''
        Stopping condition: How do we know when to stop?
        Max diameter away = d / sin(pi/n)
        d = robot diameter (self._diameter)
        n = number of robots (len(self._neighbors_ready))
        '''

        total = 0
        not_too_close = 0.5
        distances = np.array([])

        for name, neighbor in self.neighbor_position.items():
            difference = (np.array(neighbor) - np.array(self.position))/2
            distances = np.append(distances, np.linalg.norm(difference))
            if np.linalg.norm(difference) > not_too_close:
                weight = 1
            else:
                weight = 0
            total += weight * difference

        if self.complete or self._path_obstructed_neighbor or self._path_obstructed_laser:
            standings = distances <= self.stopping_distance
            if standings.all():
                if not self.complete:
                    self.get_logger().info(f"Distances = {distances}")
                    self.get_logger().info(f"stopping = {self.stopping_distance}")
                self.complete = True
                total = 0 * total
            else:
                self.complete = False
            
        self.move_direction(total)


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=4
    ## ros2 launch agent_control consensus_batch.launch.py yaml_load:=False number_robots=4

    # -p robot.id:=1 -p robot.neighbors:="[1,2,3]" -p mode.sim:=true -p laser.avoid:=true -p laser.avoid_loop_max:=2

    my_robot = None
    try:
        rclpy.init(args=args)
        my_robot = Consensus("Consensus")
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

'''
To Do:
Do sync move? - in agent file
Make a concenses example
'''