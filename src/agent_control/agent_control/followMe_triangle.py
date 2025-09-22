#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import yaml

import pdb

class FollowMe(Agent):
    def __init__(self, my_number, my_neighbors=[], formation_distance=[], *args, 
        sim=False, sync_move=False,
        at_goal_historisis=0.15,
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
        self._leader = None

        for number in my_neighbors:
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

        desired = self.neighbor_orientation[str(self._leader)]

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


def build_formation_distance(neighbor_array, my_number):
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
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    fd,_ = build_formation_distance(np.array(script_args.neighbor), script_args.index)

    try:
        rclpy.init(args=args)
        my_robot = FollowMe(int(script_args.index), np.array(script_args.neighbor), fd, sim=script_args.sim, 
            restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
