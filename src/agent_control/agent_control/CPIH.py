#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
from agent_control.TukeyMedian import TukeyContour
import traceback
import pdb

class CPIH(Agent):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.complete = False
     
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
        
        X = np.zeros((len(self.neighbor_position),2))
        i = 0
        for name, neighbor in self.neighbor_position.items():
            X[i] = np.array(neighbor) 
            
            
            # print("X[",i,"]: ",X[i])
            i = i+1
       # for neighbor in self.neighbor_poses:
        #    X[i] = np.array((self.neighbor_poses[neighbor].pose.position.x, self.neighbor_poses[neighbor].pose.position.y))
        tc = TukeyContour(X)
        if tc.median_contour.shape[0] > 0:
            # Target is the centroid of the median contour
            
            safepoint = np.mean(tc.median_contour, axis=0)
            # self.get_logger().info(f"{self.my_name} Has a valid target: {safepoint}")
        else:
            safepoint = self.position
            self.get_logger().info(f"{self.my_name} Does not have valid target.")
            self.get_logger().info(f"{tc.median_contour} ")
        target = safepoint
        if (np.linalg.norm(self.position-target)<0.3):
            self.complete = True
        self.move_to_position(target)


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=4

    ## python3 CPIH.py -i 1 -n 1 2 3 -s --ros-args -p robot.neighborhood_mode:=global -p robot.neighborhood_global:=[1,1,0,1,0,1,0,1,1] -p robot.neighborhood_size:=3
    ## ros2 run agent_control CPIH.py --ros-args --params-file src/agent_control/config/CPIH/test.yaml -p robot.id:=1 -p robot.neighbors:="[1,2,3]" -p mode.sim:=true 


    my_robot = None 

    try:
        rclpy.init(args=args)
        my_robot = CPIH("CPIH")
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