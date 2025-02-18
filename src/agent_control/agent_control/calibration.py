import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime

import pdb

class Calibration(Agent):
    def __init__(self, my_number, my_neighbors=[], *args, sim=False, sync_move=False,
        destination_tolerance=0.01,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2,
        neighbor_avoid=True, neighbor_delay=5):
        super().__init__(my_number, my_neighbors, sim=sim, sync_move=sync_move, 
                        destination_tolerance=destination_tolerance,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)

        self._mode = 0
        self._finish_cap = 10

        self._pressed_button = False
        self._robot_moving = False
        self._robot_finished = False
        self._finish_buffer = 0
        self._starting_position = None
        self._prev_position = None
        self._ending_position = None
        self._starting_rotation = None
        self._prev_rotation = None
        self._rotation_increasing = None
        self._rotation_start = False
        self._ending_roation = None
        self._num_rotations = None

    def controller(self):
        '''
        Send one command to robot and see how far that moves goes

        Needed info from agent.
        self.position                   This agents position
        self.direction_facing           The direction we are facing
        self.move_robot_()              Send command to robot
        '''

        if type(self.position) == type(None):
            return

        if not self._pressed_button:
            if not self._robot_moving:
                self._starting_position = self.position
                self._prev_position = self.position
                self._starting_rotation = self.direction_facing
                self._prev_rotation = self.direction_facing
                self._num_rotations = 0
                self._robot_finished = False

                # self.move_robot_(0.0, 0.25)
                ## 2.0, 3.0, 5.0 gave a difference of 0.95
                ## 1.5 gave a difference of 0.75
                ## 1.0 gave a difference of 0.5
                ## 0.5 gave a difference of 0.25
                ## 0.25 gave a difference 0f .145

                self.move_robot_(0.5, 0.0)
                ## 0.3, 0.45, 0.5, 1.0, and 5.0 gave a distance of 0.15
                ## 0.25, 0.4 gave a distance of 0.127
                ## 0.2 gave a distance of 0.089
                ## 0.1 gave a distance of 0.05

            else:
                print("robot is moving. Please wait for robot to stop")
            self._pressed_button = True
            

        
        if not self._robot_finished:

            if self._robot_moving:
                if (self.position == self._prev_position).all() and (self._prev_rotation == self.direction_facing).all():
                    if self._finish_cap >= self._finish_buffer:
                        self._robot_finished = True
                        self.publish_results()
                    self._finish_buffer += 1
                else:
                    self._finish_buffer = 0

            if (self._starting_position != self.position).any() or (self._starting_rotation != self.direction_facing):
                
                if type(self._rotation_increasing) == type(None):
                    if np.round(self._starting_rotation, 3) != np.round(self.direction_facing, 3):
                    # if self._starting_rotation != self.direction_facing:
                        if self._starting_rotation > self.direction_facing:
                            self._rotation_increasing = False
                        else:
                            self._rotation_increasing = True
                        self._rotation_start = True
                else:
                    if self._rotation_increasing:
                        if self.direction_facing < self._starting_rotation and self._rotation_start:
                            self._rotation_start = False
                        elif self.direction_facing >= self._starting_rotation and not self._rotation_start:
                            self._rotation_start = True
                            self._num_rotations += 1
                            print(f"{self._num_rotations} rotations completed")
                    
                    elif not self._rotation_increasing:
                        if self.direction_facing > self._starting_rotation and self._rotation_start:
                            self._rotation_start = False
                        elif self.direction_facing <= self._starting_rotation and not self._rotation_start:
                            self._rotation_start = True
                            self._num_rotations += 1
                            print(f"{self._num_rotations} rotations completed")

                self._robot_moving = True
                self._prev_position = self.position
                self._prev_rotation = self.direction_facing

    def controller2(self):
        '''
        Send one command to robot and see how fast we can cancel it

        Needed info from agent.
        self.position                   This agents position
        self.direction_facing           The direction we are facing
        self.move_robot_()              Send command to robot
        '''

        if type(self.position) == type(None):
            return

        if not self._pressed_button:
            if not self._robot_moving:
                self._starting_position = self.position
                self._prev_position = self.position
                self._starting_rotation = self.direction_facing
                self._prev_rotation = self.direction_facing
                self._num_rotations = 0
                self._robot_finished = False

                self.move_robot_(0.0, 0.5)
                ## 2.0 stop within 0.08
                ## Smaller can stop a little faster (0.5 -> 0.05)


                # self.move_robot_(0.2, 0.0)
                ## 1.0 - little inconsistent. normally stops very quick (0.01 or less)
                ## No difference lowering number

            else:
                print("robot is moving. Please wait for robot to stop")
            self._pressed_button = True
            

        
        if not self._robot_finished:

            if self._robot_moving:
                if (self.position == self._prev_position).all() and (self._prev_rotation == self.direction_facing).all():
                    if self._finish_cap >= self._finish_buffer:
                        self._robot_finished = True
                        self.publish_results()
                    self._finish_buffer += 1
                else:
                    self._finish_buffer = 0

            if (self._starting_position != self.position).any() or (self._starting_rotation != self.direction_facing):
                
                self.move_robot_(0.0, 0.0)

                self._robot_moving = True
                self._prev_position = self.position
                self._prev_rotation = self.direction_facing

    def controller3(self):
        '''
        checking to see if i delay the messages if it would run smotther.
        Did not make a difference
        '''
        max_num = 8
        if type(self._num_rotations) == type(None):
            self._num_rotations = 0
        
        if self._num_rotations >= max_num:
            self._num_rotations = 0

        if not self._num_rotations:
            self.move_robot_(0.0, 1.0)
        self._num_rotations += 1

    def publish_results(self):
        print(f"""Calibaration completed:
        Distance traveled: {np.linalg.norm(self.position - self._starting_position)}
        Rotation direction: {"increasing" if self._rotation_increasing else "Decreasing"}
        Starting and Ending rotation: {self._starting_rotation}, {self.direction_facing}
        Rotations traveled: {self._num_rotations}
        Degree difference: {self.direction_facing - self._starting_rotation}""")





def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py yaml_load:=False robot_number:=2
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Mode you want to calibrate in")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    print(f"Press c to issue move command")

    rclpy.init(args=args)
    my_robot = Calibration(
        int(script_args.index), 
        np.array(script_args.neighbor), 
        sim=script_args.sim,
        laser_avoid=True, neighbor_avoid=True,)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

