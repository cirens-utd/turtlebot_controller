#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
from path_plan2 import Path
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
class PS(Enum):
    PATH_OBSTRUCTED = 0
    PATH_CLEAR = 1
    
class Path_Test(Agent):
    def __init__(self, my_number, my_neighbors=[], end_target=[], *args, 
        sim=False, sync_move=False, logging=False, angle_tolerance=0.2,
        restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
        destination_tolerance=0.01,at_goal_historisis = 0.01,
        laser_avoid=False, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
        neighbor_avoid=False, neighbor_delay=5):
        '''
        formation_distance should be in the following formate
        formation_distance = {
            "neighbor#": 2.0,
            ...
        }
        '''
        super().__init__(my_number, my_neighbors, sync_move=sync_move, sim=sim,
                        destination_tolerance=destination_tolerance, logging=logging, angle_tolerance=angle_tolerance, at_goal_historisis=at_goal_historisis,
                        restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)
        
        self.end_target = end_target
        self.path = Path()
        self.subgoals = []
        self.subgoal_complete = False
        self.subgoal_idx = 0
        self.start = True
        self.destination_tolerance = destination_tolerance
        self.led_pub = self.create_publisher(LightringLeds, '/'+self.my_name+'/cmd_lightring', qos_profile_sensor_data)
        self.grid_radius = 5
        self.ps = PS.PATH_CLEAR
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
        obstacles = []
        for name, neighbor in self.neighbor_position.items():
            if np.linalg.norm(self.position-neighbor)<self.grid_radius:
                obstacles.append(neighbor)
        for subgoal in self.subgoals:
            for obstacle in obstacles:
                if subgoal == obstacle:
                    self.ps = PS.PATH_OBSTRUCTED
            
        # self.get_logger().info(f"neighbors_complete:{self.neighbors_complete}")
        
        if self.start: 
            self.path.update_costmap(self.position,obstacles)
            self.get_logger().info(f"endtarget: {self.end_target}")
            self.subgoals = self.path.astar(self.position, self.end_target)
            self.get_logger().info(f"subgoals: {self.subgoals}")
            self.start = False
        subgoal_distance = abs(np.linalg.norm(self.subgoals[self.subgoal_idx]-self.position))
        if subgoal_distance < self.destination_tolerance:
            self.subgoal_complete = True

        if self.subgoal_complete and self.subgoal_idx < len(self.subgoals):
            self.led_state(2)
            self.subgoal_idx +=1
            self.subgoal_complete = False
        elif self.ps == PS.PATH_OBSTRUCTED:
            self.led_state(0)
            self.path.update_costmap(obstacles)
            self.subgoals = path.astar(self.position,self.end_target)
            self.subgoal_idx =0
            self.led_state(1)
            self.move_direction= self.subgoals[self.subgoal_idx]-self.position
        else:
            self.led_state(1)
            self.move_direction= self.subgoals[self.subgoal_idx]-self.position
        start = False
        
        
        if self.suboal_idx == len(self.subgoals) and self.subgoal_complete:
            self.led_state(3)
        


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
            case 3: 
                for led in lightring_msg.leds:
                    led.red = 0
                    led.green = 255
                    led.blue = 255
        self.led_pub.publish(lightring_msg)


            
    
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
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-f", "--formation", type = str, default = "/home/ubuntu/Turtlebot_Controller/src/agent_control/config/Goals/",help = "/path/to/agent_setup.yaml")
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

    yaml_data = get_yaml(f"/home/ubuntu/Turtlebot_Controller/src/agent_control/config/goals/agent_goals.yaml")
    goal = yaml_data[f"agent_goal{script_args.index}"]
    
    #neighbor = yaml_data["Adjacency_Matrix"][script_args.index]    
    neighbor = script_args.neighbor


    rclpy.init(args=args)
    my_robot = Path_Test(int(script_args.index), np.array(neighbor), goal, sim=script_args.sim, 
        restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
