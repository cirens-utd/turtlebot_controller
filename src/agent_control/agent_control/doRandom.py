#!/usr/bin/env python3

import rclpy
from agent_control.agent import Agent
import numpy as np
import datetime
import traceback
import pdb


class DORANDOM(Agent):
    def __init__(self, node_name):
        self.extra_param_update_map = {
            "Random.distance": "distance",
            "Random.time": "time",
            "Random.num_points": "num_points"
        }
        super().__init__(node_name)
        self.declare_parameter("Random.distance", 2)    # Distance you want your points way from each other
        self.distance = self.get_parameter("Random.distance").value
        self.declare_parameter("Random.time", 3)    # number of second you attempt to drive to the location. 0 is no limit
        self.time = self.get_parameter("Random.time").value
        self.declare_parameter("Random.num_points", 4)    # number of points you want it your random movement
        self.num_points = self.get_parameter("Random.num_points").value

        self.dest_index = 0
        self.starting_place = None
        self.destination = []
        self.end_time = None

    def build_destination(self):
        for i in range(self.num_points):
            angle = 2 * np.pi * i / self.num_points
            x = self.starting_place[0] + self.distance * np.cos(angle)
            y = self.starting_place[1] + self.distance * np.sin(angle) 
            self.destination.append((round(x, 2), round(y, 2)))
            
        np.random.shuffle(self.destination)
        if self.time:
            self.end_time = datetime.datetime.now() + datetime.timedelta(seconds=self.time)

        self.get_logger().info(f"{self.my_name}: Points to travel to = {self.destination}")

    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our concensus logic here.

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''
        
        if type(self.starting_place) == type(None):
            self.starting_place = self.position
            self.build_destination()

        if self.destination_reached or (type(self.end_time) != type(None) and datetime.datetime.now() > self.end_time):
            self.dest_index += 1
            if len(self.destination) <= self.dest_index:
                self.dest_index = 0
            if self.time:
                self.end_time = datetime.datetime.now() + datetime.timedelta(seconds=self.time)  

        self.move_to_position(self.destination[self.dest_index])


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=2
    ## ros2 run agent_control doRandom.py --ros-args -p robot.id:=1 -p robot.neighbors:='[1,2]' -p mode.sim:=true 

    my_robot = None 

    try:
        rclpy.init(args=args)
        my_robot = DORANDOM("DoRandom")
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
