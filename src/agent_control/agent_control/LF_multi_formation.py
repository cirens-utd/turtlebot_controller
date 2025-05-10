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

class Led_state(Enum):
    READY = 0
    MOVING = 1
    COMPLETE = 2
    
class LF_multi_formation(Agent):
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
        
        self.formation_distance = formation_distance
        #Added for multiple formations
        if isinstance(formation_distance,list):
            self._formation_list = formation_distance
            self._formation_idx = 0
            self._formation_distance = self._formation_list[self._formation_idx]
        self.led_pub = self.create_publisher(LightringLeds, '/'+self.my_name+'/cmd_lightring', qos_profile_sensor_data)

        for number in my_neighbors:
            if str(number) not in self._formation_distance:
                print(f"Neighbors: {my_neighbors}")
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
        self.get_logger().info(f"neighbors_complete:{self.neighbors_complete}")
        if self.neighbors_complete and self._formation_idx < len(self._formation_list):
            self.led_state(2)
            self._formation_idx += 1
            self._formation_distance = self._formation_list[self._formation_idx]
            self.get_logger().info(f"formation index: {self._formation_idx}")
            self.get_logger().info(f"formation_list: {self._formation_list}")
            self.neighbors_complete = False
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
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-f", "--formation", type = str, default = "/home/ubuntu/Turtlebot_Controller/src/agent_control/config/CiRENS_formation/",help = "/path/to/agent_setup.yaml")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()
    
    ## Changed yaml load to take in a directory name and then load all yaml files from that directory 
    ## Another way would have  been to just put multiple formations in the same yaml, but this may make things easier if we
    ## want to add new formations or change adjacency matrices in the future.  Also, right now the agent's neighborhood
    ## will end up being the same no matter the yaml file. We probably want to adjust this in the future. 
    yaml_data = []
    list = ['1','2','3','4']
    for num in list:
        yaml_data.append(get_yaml(f"/home/ubuntu/Turtlebot_Controller/src/agent_control/config/shapes/agent_setup({num}).yaml"))
    # for file in os.listdir(script_args.formation):
    #     if file.endswith(".yaml") or file.endswith(".yml"):
    #         try:
    #             yaml_data.append(get_yaml(os.path.join(script_args.formation, file)))
                
    #         except yaml.YAMLError as exc:
    #             print(exc)
    #             break
    if len(yaml_data)>1:
        fd_list = []
        neighbor_list = []
        for data in yaml_data:
            fd,neighbor = build_formation_distance(data, script_args.index, script_args.neighbor)
            fd_list.append(fd)
    else:
         ##This should remain the same as the original code if there is only one yaml file
        fd, neighbor = build_formation_distance(yaml_data[0], script_args.index, script_args.neighbor)
        fd_list = fd

    rclpy.init(args=args)
    my_robot = LF_multi_formation(int(script_args.index), np.array(neighbor), fd_list, sim=script_args.sim, 
        restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
