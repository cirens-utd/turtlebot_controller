#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import yaml
import os
import pdb
from irobot_create_msgs.msg import LightringLeds
from rclpy.qos import qos_profile_sensor_data
from enum import Enum
import traceback

class Led_state(Enum):
    READY = 0
    MOVING = 1
    COMPLETE = 2
    
class LF_multi_formation(Agent):
    def __init__(self, node_name, yaml_data={}):
        '''
        formation_distance should be in the following formate
        formation_distance = {
            "neighbor#": 2.0,
            ...
        }
        '''
        super().__init__(node_name)

        if len(yaml_data)>1:
            fd_list = []
            neighbor_list = []
            for data in yaml_data:
                formation_distance,neighbor,all_fd = self.build_formation_distance(data, self.my_number, self._my_neighbors)
                fd_list.append(formation_distance)
        else:
            ##This should remain the same as the original code if there is only one yaml file
            formation_distance, neighbor = self.build_formation_distance(yaml_data[0], self.my_number, self._my_neighbors)
            fd_list = formation_distance
        
        self._my_neighbors = neighbor
        self._rebuild_neighborhood()

        self.formation_distance = formation_distance
        self.all_fd = fd_list

        #Added for multiple formations
        if isinstance(formation_distance,list):
            self._formation_list = formation_distance
            self._formation_idx = 0
            self._formation_distance = self._formation_list[self._formation_idx]
        self.led_pub = self.create_publisher(LightringLeds, '/'+self.my_name+'/cmd_lightring', qos_profile_sensor_data)

        self.complete_counter = 0
        self.new_counter = 0
        self.new_formation = False
        self.delay_cycles = [150, 150, 150, None]
        self.next_formation = False

        for number in self._my_neighbors:
            if str(number) not in self._formation_distance:
                print(f"Neighbors: {self._my_neighbors}")
                print(f"Formation_distance: {self._formation_distance}")
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
        # self.get_logger().info(f"neighbors_complete:{self.neighbors_complete}")
        if self.next_formation and self._formation_idx < len(self._formation_list):
            if True:
                self.led_state(2)
                self._formation_idx += 1
                self._formation_distance = self._formation_list[self._formation_idx]
                self.get_logger().info(f"formation index: {self._formation_idx}")
                self.get_logger().info(f"formation_list: {self._formation_list}")
                self.neighbors_complete = False
                self.next_formation = False
        elif self.path_obstructed:
            self.led_state(0)
        else:
            self.led_state(1)

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
            self.led_state(1)

        if not self.next_formation and type(self.delay_cycles[self._formation_idx]) != type(None):
            if self.complete_counter > self.delay_cycles[self._formation_idx]:
                self.next_formation = True
                self.complete_counter = 0
            else:
                self.complete_counter += 1

        self.get_logger().info(f"{self.my_name}: My Index Number is: {self._formation_idx}\n my_counter: {self.complete_counter}\n What: {self.next_formation}")

    def led_state(self, state):
        lightring_msg = LightringLeds()
        lightring_msg.header.stamp = self.get_clock().now().to_msg()
        lightring_msg.override_system = True
        match state:
            case 0:
                for led in lightring_msg.leds:
                    led.red = 255
                    led.green =0
                    led.blue = 0
            case 1: 
                for led in lightring_msg.leds:
                    led.red = 0
                    led.green =0
                    led.blue = 255
            case 2:
                for led in lightring_msg.leds:
                    led.red = 0
                    led.green =255
                    led.blue = 0
        self.led_pub.publish(lightring_msg)

    def really_complete(self):
        for name, neighbor in self.neighbor_position.items():
            for name1, neighbor1 in self.neighbor_position.items():
                distance  = self.all_fd[str(name)][str(name1)]
                actual_distance = np.linalg.norm(np.array(neighbor) - np.array(neighbor1))
                if np.abs(actual_distance - distance) > self._destination_tolerance:
                    self.get_logger().info(f"Distance Desired: {distance}\t What I got: {actual_distance}")
                    return False
            distance  = self.all_fd[str(name)][str(self.my_number)]
            actual_distance = np.linalg.norm(np.array(neighbor) - np.array(self.position))
            if np.abs(actual_distance - distance) > self._destination_tolerance:
                self.get_logger().info(f"Distance Desired: {distance}\t What I got: {actual_distance}")
                return False
        return True

    def build_formation_distance(self, data, my_number, input_neighbors):
        fd = {}
        set_index = -1
        neighbor = []
        all_fd = {}


        for index, number in enumerate(input_neighbors):
            if number == my_number:
                set_index = index
            else:
                neighbor.append(number)
                all_fd[str(number)] = {}
                for idx, num in enumerate(input_neighbors):
                    all_fd[str(number)][str(num)] = data['formation_distances'][index][idx]
        
        if set_index != -1:
            for index, number in enumerate(input_neighbors):
                fd[str(number)] = data['formation_distances'][set_index][index]
            
            return fd, neighbor, all_fd
        raise NotImplementedError("Attempted to start LF_Formation Node but the number given in neighbor argument didn't match the size of the formation matrix.")   
    
def get_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


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
    
    ## Changed yaml load to take in a directory name and then load all yaml files from that directory 
    ## Another way would have  been to just put multiple formations in the same yaml, but this may make things easier if we
    ## want to add new formations or change adjacency matrices in the future.  Also, right now the agent's neighborhood
    ## will end up being the same no matter the yaml file. We probably want to adjust this in the future. 
    yaml_data = []
    # 1 is circle
    # 3 is star
    # 2 is square
    # 4 is smiley
    list = ['1', '3', '2', '4']
    for num in list:
        # yaml_data.append(get_yaml(f"/home/ubuntu/Turtlebot_Controller/src/agent_control/config/shapes/agent_setup({num}).yaml"))
        yaml_data.append(get_yaml(f"src/agent_control/config/shapes/agent_setup({num}).yaml"))
    # for file in os.listdir(script_args.formation):
    #     if file.endswith(".yaml") or file.endswith(".yml"):
    #         try:
    #             yaml_data.append(get_yaml(os.path.join(script_args.formation, file)))
                
    #         except yaml.YAMLError as exc:
    #             print(exc)
    #             break

    try:
        rclpy.init(args=args)
        my_robot = LF_multi_formation("LF_Multi_Formation", yaml_data)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        try:
            my_robot.shutdown()
            rclpy.shutdown()
        except Exception as e:
            my_robot.get_logger().info(f"Error shutting down. {e}")

if __name__ == '__main__':
    main()
