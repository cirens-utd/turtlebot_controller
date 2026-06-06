#!/usr/bin/env python3

import rclpy
from agent_control.agent import Agent
import numpy as np
import traceback
import pdb


class DODRIVEAWAY(Agent):
    def __init__(self, node_name):
        self.extra_param_update_map = {
            "DriveAway.magnitude": "magnitude"
        }
        super().__init__(node_name)
        self.declare_parameter("DriveAway.magnitude", 2)    # Magnitude of driving away
        self.magnitude = self.get_parameter("DriveAway.magnitude").value
     
    def get_direction(self, start, end):
        start_arr = np.array(start)
        end_arr = np.array(end)
        
        direction = start_arr - end_arr
        magnitude = np.linalg.norm(direction)
        
        # Guard against division by zero if points are identical
        if magnitude == 0:
            return np.zeros_like(direction, dtype=float)
        unit_vector = direction / magnitude
        return unit_vector

    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our concensus logic here.

        Equation:
        drive away from concensus

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''
        total = 0
        for name, neighbor in self.neighbor_position.items():
            difference = (np.array(neighbor) - np.array(self.position))/2
            total +=  difference

        self.move_direction(self.get_direction(self.position, total) * self.magnitude)


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=2
    ## ros2 run agent_control doDriveAway.py --ros-args -p robot.id:=2 -p robot.neighbors:='[1,2]' -p mode.sim:=true 

    my_robot = None 

    try:
        rclpy.init(args=args)
        my_robot = DODRIVEAWAY("DoDriveAway")
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
