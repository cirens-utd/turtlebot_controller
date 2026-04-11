#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import yaml

import pdb

class FollowMe(Agent):
    def __init__(self, node_name):
        '''
        formation_distance should be in the following formate
        formation_distance = {
            "neighbor#": 2.0,
            ...
        }
        '''
        super().__init__(node_name)

        formation_distance,_ = self.build_formation_distance(self._my_neighbors, self.my_number)
        self._formation_distance = formation_distance
        self._leader = None

        for number in self._my_neighbors:
            if type(self._leader) == type(None) or number > self._leader:
                self._leader = number
                
            if str(number) not in formation_distance:
                raise NotImplementedError('When Passing formation distance into LF_Formation, all neighbors must have a set distance')

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

        start = False
        total = [0,0]
        tolerance = 0.1

        for name, neighbor in self.neighbor_position.items():
            start = True
            difference = np.array(neighbor) - np.array(self.position)
            if abs(np.linalg.norm(difference) - self._formation_distance[str(name)])  > tolerance:
                total += (np.linalg.norm(difference) - self._formation_distance[str(name)]) * difference
        if start:
            self.move_direction(total)

    def end_controller(self):
        # want to copy the followers angle
        # use self.neighbor_orientation[name]

        if self.my_number != self._leader:

            desired = self.get_angle_quad(self.neighbor_orientation[self._leader])

            # Need to try this method
            '''
            Simplified Kuramoto Coupled Oscillator Model

            Phi,i = - SUM(sin(Phi,i - phi,j))
            '''
            # match_heading = 0
            # for name, neighbor in self.neighbor_orientation.items():
            #     match_heading += np.sin(self.direction_heading - neighbor)
            # I think this would require a move by set angle function. Aka, move 25 Radians and not to a set position like below.?

            # desired /= num_neighbors
            self.move_to_angle(desired)
        else:
            self.robot_status = "FINISHED"
            self.shutdown()
            rclpy.shutdown()

    def build_formation_distance(self, neighbor_array, my_number):
        fd = {}
        set_index = -1
        neighbor = []

        # distances for 5 nodes
        distances = [[0.00, 1.81, 2.91, 2.98, 1.76],[1.81, 0.00, 2.07, 2.96, 2.95],[2.91, 2.07, 0.00, 1.36, 2.75],[2.98, 2.96, 1.36, 0.00, 1.99],[1.76, 2.95, 2.75, 1.99, 0.00]]

        for index, number in enumerate(neighbor_array):
            neighbor.append(number)
            if number == my_number:
                set_index = index
        
        if set_index != -1:
            for index, number in enumerate(neighbor_array):
                fd[str(number)] = distances[set_index][index]
            
            return fd, neighbor
        
        raise ValueError(f"My Index Value was not passed into the as one of the Neighbors")

def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=3
    ## ros2 launch agent_control ...
    ## Note: Need to edit config/agent_setup/agent_setup.yaml
    '''
    You formation yaml should have robot numbers in it and the formation distances.
    You pass in which node is this one through -i and all the others will be neighbors
    '''

    # --ros-args -p robot.id:=1 -p robot.neighbors:="[1,2,3]" -p mode.sim:=true -p laser.avoid:=true -p laser.avoid_loop_max:=2nt, help="Index of this robot")

    my_robot = None
    try:
        rclpy.init(args=args)
        my_robot = FollowMe("FollowMe")
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
