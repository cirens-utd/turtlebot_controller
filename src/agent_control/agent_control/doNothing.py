#!/usr/bin/env python3

import rclpy
from agent_control.agent import Agent
import traceback
import pdb


class DONOTHING(Agent):
    def __init__(self, node_name):
        super().__init__(node_name)
     
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
        
        self.move_to_position(self.position)


def main(args=None):
    my_robot = None 

    try:
        rclpy.init(args=args)
        my_robot = DONOTHING("DoNothing")
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
