#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
from agent_control.TukeyMedian import TukeyContour, CPIH
import traceback
import pdb

class CPIH(Agent):
    def __init__(self, node_name):
        self._extra_param_update_map = {
            "CPIH.self_trust": "self_trust",
            "CPIH.safe_point_mode": "safe_point_mode",
            "CPIH.imprecision": "imprecision"
        }
        super().__init__(node_name)
        self.complete = False

        self.declare_parameter("CPIH.self_trust", 1)    # 0 - Self Distrust, 1 - normal Tukey, 2 = Self Trust
        self.self_trust = self.get_parameter("CPIH.self_trust").value

        self.declare_parameter("CPIH.safe_point_mode", 0)    # 0 - Use Tukey Centroid, 1 - Use Safepoint, 2 - use fByzantine-safe point
        self.safe_point_mode = self.get_parameter("CPIH.safe_point_mode").value

        self.declare_parameter("CPIH.imprecision", 0)
        self.imprecision = self.get_parameter("CPIH.imprecision").value

    def getImprecisionRegions(self, X,imp):
        n = len(X)
        Bx= np.zeros((n,4,2))
        for i in range(len(X)):
            Bx[i,0,:]= [X[i][0]-imp, X[i][1]+imp]
            Bx[i,1,:]= [X[i][0]+imp, X[i][1]+imp]
            Bx[i,2,:]= [X[i][0]+imp, X[i][1]-imp]
            Bx[i,3,:]= [X[i][0]-imp, X[i][1]-imp]
        return(Bx)

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
        
        X = np.zeros((len(self.neighbor_position)+1,2))
        i = 1
        X[0] = self.position
        for name, neighbor in self.neighbor_position.items():
            X[i] = np.array(neighbor) 
            
            
            # print("X[",i,"]: ",X[i])
            i = i+1
        Bx = self.getImprecisionRegions(X,self.imprecision)

        if self.safe_point_mode == 0:
            # for neighbor in self.neighbor_poses:
            #    X[i] = np.array((self.neighbor_poses[neighbor].pose.position.x, self.neighbor_poses[neighbor].pose.position.y))
            tc = TukeyContour(X)

            '''
            This is finding the mean of the tukey median. This is the mean of the vertices of the deepest area in the set.
            '''
            if tc.median_contour.shape[0] > 0:
                # Target is the centroid of the median contour
                
                safepoint = np.mean(tc.median_contour, axis=0)
                # self.get_logger().info(f"{self.my_name} Has a valid target: {safepoint}")
            else:
                safepoint = self.position
                self.get_logger().info(f"{self.my_name} Does not have valid target.")
                self.get_logger().info(f"{tc.median_contour} ")
            target = safepoint
        elif self.safe_point_mode == 1:
            sp = CPIH()
            target = sp.CPIH_Safepoint(Bx, 0, self.position, mode=self.self_trust)
        elif self.safe_point_mode == 2:
            target = sp.CPIH_Fast_Safepoint(Bx, 0, self.position, mode=self.self_trust)
        else:
            target = self.position
            self.get_logger().warning(f"{self.my_name}: Invalid mode set. Mode = {self.safe_point_mode}")

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