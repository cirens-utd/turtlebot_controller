#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import yaml

import pdb

class LF_Formation(Agent):
    def __init__(self, my_number, my_neighbors=[], formation_distance=[], *args, 
        sim=False, sync_move=False,
        restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
        destination_tolerance=0.01,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
        neighbor_avoid=True, neighbor_delay=5):
        '''
        formation_distance should be in the following formate
        formation_distance = {
            "neighbor#": 2.0,
            ...
        }
        '''
        super().__init__(my_number, my_neighbors, sync_move=sync_move, sim=sim,
                        destination_tolerance=destination_tolerance,
                        restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)
        self._formation_distance = formation_distance

        for number in my_neighbors:
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

def build_formation_distance(data, my_number):
    fd = {}
    set_index = -1
    neighbor = []

    for index, number in enumerate(data['robot_numbers']):
        if number == my_number:
            set_index = index
        else:
            neighbor.append(number)
    
    if set_index != -1:
        for index, number in enumerate(data['robot_numbers']):
            fd[str(number)] = data['formation_distances'][set_index][index]
        
        return fd, neighbor
    raise NotImplementedError("Attempted to start LF_Formation Node but the number given didn't match a number in the yaml.")


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=6
    ## ros2 launch agent_control lf_formation.launch.py sim_mode:=True
    ## Note: Need to edit config/agent_setup/agent_setup.yaml
    '''
    You formation yaml should have robot numbers in it and the formation distances.
    You pass in which node is this one through -i and all the others will be neighbors
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-f", "--formation", type = str, default = "path/to/ws/turtlebot_simulator/src/agent_control/config/agent_setup/agent_setup.yaml",help = "/path/to/agent_setup.yaml")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    yaml_data = get_yaml(script_args.formation)
    fd, neighbor = build_formation_distance(yaml_data, script_args.index)

    rclpy.init(args=args)
    my_robot = LF_Formation(int(script_args.index), np.array(neighbor), fd, sim=script_args.sim, 
        restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
