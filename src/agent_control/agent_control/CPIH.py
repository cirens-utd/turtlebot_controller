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

        # self.declare_agent_parameters()
        # self.assign_parameters()
        # self.add_on_set_parameters_callback(self.parameter_callback)

        # # setting neighborhood flags
        # if len(self._neighborhood) > 0:
        #     # If value is not set, use default
        #     for robot, values in self.neighbor_poses.items():
        #         self.neighbor_poses[robot]["in_neighborhood"] = bool(self._neighborhood_default)
        # # Given global neighborhood
        # if self._neighborhood_mode == 'global':
        #     # Need to find what index we are looking at
        #     my_index = -1
        #     for index, neighbor in enumerate(my_neighbors):
        #         if neighbor == my_number:
        #             my_index = index
        #             break
        #     if my_index < 0 or my_index >= len(self._neighborhood):
        #         if my_index < 0:
        #             self.get_logger().warning(f"Neighborhood mode set to global but my index was not included in nieghbor layout\\nNeighbors: {my_neighbors}")
        #         else:
        #             self.get_logger().warning(f"Neighborhood mode set to global but my index was outside the range of the matrix given\nIndex: {my_index}\nNeighborhood: {self._neighborhood}")
        #         # all values go to default
        #         for robot, values in self.neighbor_poses:
        #             self.neighbor_poses[robot]["in_neighborhood"] = bool(self._neighborhood_default)
        #     else:
        #         # loop trough each of the neighbors and set the value
        #         for index, neighbor in enumerate(my_neighbors):
        #             # Insure vlaues are actually in neighbors
        #             if str(neighbor) in self.neighbor_poses:
        #                 if index < len(self._neighborhood[my_index]):
        #                     self.neighbor_poses[str(neighbor)]["in_neighborhood"] = bool(self._neighborhood[my_index][index])
        #                 else:
        #                     self.get_logger().warning(f"Neighbor {neighbor} at index {index} is outside the range of neighborhood.\n{self._neighborhood}")
        #                     self.neighbor_poses[str(neighbor)]["in_neighborhood"] = bool(self._neighborhood_default)
        # else:
        #     # loop trough each of the neighbors and set the value
        #     for index, neighbor in enumerate(my_neighbors):
        #         # Insure vlaues are actually in neighbors
        #         if str(neighbor) in self.neighbor_poses:
        #             if index < len(self._neighborhood):
        #                 self.neighbor_poses[str(neighbor)]["in_neighborhood"] = bool(self._neighborhood[index])
        #             else:
        #                 self.get_logger().warning(f"Neighbor {neighbor} at index {index} is outside the range of neighborhood.\n{self._neighborhood}")
        #                 self.neighbor_poses[str(neighbor)]["in_neighborhood"] = bool(self._neighborhood_default)

        # # # Only neighborhood in neighbor_position
        # # for name, neighbor in self.neighbor_poses.items():
        # #     if not neighbor["in_neighborhood"] and name in self.neighbor_position:
        # #         self.neighbor_position.pop(name, None)
        # #         print(f"Poped {name}")

    # def declare_agent_parameters(self):
    #     '''
    #         Converting exisiting system
    #         /**:
    #             ros__parameters:
    #                 robot:
    #                     id: 1
    #                     neighbors: [2, 3]
    #                     diameter: 0.4

    #                 mode:
    #                     sim: false
    #                     sync_move: false
    #                     viewer: false

    #                 logging:
    #                     enabled: true
    #                     paused: false
    #                     log_dict_length: 9000
    #                     config_file: "turtlebot_global_config.yaml"

    #                 sensors:
    #                     use_mocap: true
    #                     use_camera: false

    #                 motion:
    #                     max_speed: 2.0
    #                     min_speed: 0.5
    #                     max_angle: 2.0
    #                     min_angle: 0.1
    #                     destination_tolerance: 0.01
    #                     angle_tolerance: 0.1
    #                     at_goal_historisis: 1
    #                     driving_heading_tolerance: 0.785   # pi/4

    #                 timing:
    #                     control_loop_period: 0.1
    #                     log_loop_period: 0.1

    #                 restricted_area:
    #                     enabled: false
    #                     x_min: -2.9
    #                     x_max: 2.9
    #                     y_min: -5.0
    #                     y_max: 4.0

    #                 laser:
    #                     enabled: true
    #                     distance: 0.5
    #                     delay: 5
    #                     walk_around: 2
    #                     avoid_loop_max: 1

    #                 neighbor:
    #                     enabled: true
    #                     delay: 5
    #                     tolerance: 0.5
    #                     walk_around: true
    #     '''
    #     '''
    #     robot:
    #         neighborhood_mode: "local"   # "local" or "global" -> Local only has my neighborhood. Global has all neighborhoods
    #         neighborhood_local: []
    #         neighborhood_global: []
    #         neighborhood_size: 3
    #         neighborhood_default: 1     # if outside the span, 1 means they are a neighbor and 0 means they are not
    #     '''

    #     self.declare_parameter("robot.neighborhood_mode", 'local')
    #     self.declare_parameter("robot.neighborhood_local", [1])
    #     self.declare_parameter("robot.neighborhood_global", [1])
    #     self.declare_parameter("robot.neighborhood_size", 1)
    #     self.declare_parameter("robot.neighborhood_default", 1)
    
    # def assign_parameters(self):
    #     self._neighborhood_mode = self.get_parameter("robot.neighborhood_mode").value
    #     self._neighborhood_default = self.get_parameter("robot.neighborhood_default").value
    #     if self._neighborhood_mode == 'global':
    #         flat = self.get_parameter("robot.neighborhood_global").value
    #         flat = [int(x) for x in flat] 
    #         self._neighborhood_size = self.get_parameter("robot.neighborhood_size").value
    #         self._neighborhood = [flat[i*self._neighborhood_size:(i+1)*self._neighborhood_size] for i in range(self._neighborhood_size)]
    #     elif self._neighborhood_mode == 'local':
    #         self._neighborhood = self.get_parameter("robot.neighborhood_local").value
    #     else:
    #         self.get_logger().warning(f"Neighborhood value was in valid: {self._neighborhood_mode}. Will us NO neighborhood")
    #         self._neighborhood = []
    

    # # Allows for live updates
    # def parameter_callback(self, params):
    #     '''
    #         example: ros2 param set /NodeName motion.max_speed 1.5
    #         May want to do live updates:
    #             motion.max_speed
    #             motion.min_speed
    #             motion.max_angle
    #             motion.min_angle
    #             motion.destination_tolerance
    #             motion.angle_tolerance
    #             motion.at_goal_historisis
    #             motion.driving_heading_tolerance
    #             laser.distance
    #             laser.delay
    #             laser.walk_around
    #             laser.avoid_loop_max
    #             neighbor.delay
    #             neighbor.tolerance
    #             neighbor.walk_around
    #             restricted_area.enabled
    #             restricted_area.x_min
    #             restricted_area.x_max
    #             restricted_area.y_min
    #             restricted_area.y_max
    #             logging.enabled
    #     '''
    #     # for param in params:
    #     #     if param.name == "tags.tags.tag_size":
    #     #         self.tag_size = param.value
    #     #     elif param.name == "tags.x_ratio":
    #     #         self.x_ratio = param.value
    #     #     elif param.name == "tags.display_image":
    #     #         self.display_image = param.value
    #     return rclpy.parameter.ParameterEventHandler.Result(successful=True)
        
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
            
            
            print("X[",i,"]: ",X[i])
            i = i+1
       # for neighbor in self.neighbor_poses:
        #    X[i] = np.array((self.neighbor_poses[neighbor].pose.position.x, self.neighbor_poses[neighbor].pose.position.y))
        tc = TukeyContour(X)
        if tc.median_contour.shape[0] > 0:
            # Target is the centroid of the median contour
            
            safepoint = np.mean(tc.median_contour, axis=0)
            self.get_logger().info(f"{self.my_name} Has a valid target: {safepoint}")
        else:
            safepoint = self.position
            self.get_logger().info(f"{self.my_name} Does not have valid target.")
            self.get_logger().info(f"{tc.median_contour} ")
        target = safepoint
        if (np.linalg.norm(self.position-target)<0.3):
            self.complete = True
        if (self.my_number != 3):
            self.move_to_position(target)
        else:
            self.move_to_position(np.array(([0, -4])))


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
            rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
To Do:
Do sync move? - in agent file
Make a concenses example
'''