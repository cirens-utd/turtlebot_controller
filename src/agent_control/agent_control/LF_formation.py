#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import yaml
import traceback

import pdb

class LF_Formation(Agent):
    def __init__(self, node_name, yaml_data={}):
        '''
        formation_distance should be in the following formate
        formation_distance = {
            "neighbor#": 2.0,
            ...
        }
        '''
        super().__init__(node_name)

        formation_distance, neighbor = build_formation_distance(yaml_data, self.my_number, self._my_neighbors)
        self._my_neighbors = neighbor
        self._rebuild_neighborhood()

        self._formation_distance = formation_distance

        for number in self._my_neighbors:
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

def get_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def build_formation_distance(data, my_number, input_neighbors):
    fd = {}
    set_index = -1
    neighbor = []


    for index, number in enumerate(input_neighbors):
        if number == my_number:
            set_index = index
        else:
            neighbor.append(number)
    
    if set_index != -1:
        for index, number in enumerate(input_neighbors):
            fd[str(number)] = data['formation_distances'][set_index][index]
        
        return fd, neighbor
    raise NotImplementedError("Attempted to start LF_Formation Node but the number given in neighbor argument didn't match the size of the formation matrix.")


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=6
    ## ros2 launch agent_control lf_formation.launch.py sim_mode:=True
    ## Note: Need to edit config/agent_setup/agent_setup.yaml
    '''
    You formation yaml should have robot numbers in it and the formation distances.
    You pass in which node is this one through -i and all the others will be neighbors
    This requires that all the robots in teh system are passed in neighbor, including our index
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--formation", type = str, default = "src/agent_control/config/agent_setup/agent_setup.yaml",help = "/path/to/agent_setup.yaml")
    script_args, ros_args = parser.parse_known_args()

    yaml_data = get_yaml(script_args.formation)

    try:
        rclpy.init(args=ros_args)
        my_robot = LF_Formation("LF_Formation", yaml_data)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
