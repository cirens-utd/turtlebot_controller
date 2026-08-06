#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
from geometry_msgs.msg import PoseStamped
import argparse
import datetime
from agent_control.TukeyMedian import TukeyContour, SafePoint
import traceback
import pdb

class CPIH(Agent):
    def __init__(self, node_name):
        self.extra_param_update_map = {
            "CPIH.self_trust": "self_trust",
            "CPIH.safe_point_mode": "safe_point_mode",
            "CPIH.imprecision": "imprecision",
            "CPIH.push_bad": "push_bad"
        }
        self.extra_log_field_map = {
            'safe_area': '_safe_area',
            'tukey_depth': '_tukey_depth',
            'center_depth': '_center_depth',
            'self_trust': 'self_trust',
            'safe_point_mode': 'safe_point_mode',
            'imprecision': 'imprecision'
        }
        super().__init__(node_name)
        self._safe_area = []
        self._tukey_depth = 0
        self._center_depth = 0

        self.complete = False

        self.declare_parameter("CPIH.self_trust", 1)    # 0 - Self Distrust, 1 - normal Tukey, 2 = Self Trust
        self.self_trust = self.get_parameter("CPIH.self_trust").value

        self.declare_parameter("CPIH.safe_point_mode", 0)    # 0 - Centerpoint 1 - Use Tukey Centroid 2 - use fByzantine-safe point
        self.safe_point_mode = self.get_parameter("CPIH.safe_point_mode").value

        self.declare_parameter("CPIH.imprecision", 0.0)
        self.imprecision = self.get_parameter("CPIH.imprecision").value

        self.declare_parameter("CPIH.push_bad", False)  # Send agents over 10 to the far -y direction
        self.push_bad = self.get_parameter("CPIH.push_bad").value

        # Sanity Check
        self.get_logger().info(f"{self.my_name} Running Point Mode: {self.safe_point_mode}")
        self.get_logger().info(f"{self.my_name} Running trust Mode: {self.self_trust}")


    def neighbor_pose_callback_(self, pose: PoseStamped, name):
        # Make bad guys go to the moon
        new_pose = pose
        if self.push_bad and int(name) > 10:
            # new_pose.pose.position.x = (np.abs(pose.pose.position.x) + 100 ) * np.abs(pose.pose.position.x)/pose.pose.position.x
            new_pose.pose.position.x = pose.pose.position.x
            new_pose.pose.position.y = (np.abs(pose.pose.position.y) + 100 ) * np.abs(pose.pose.position.y)/pose.pose.position.y

        self.update_neighbor_position_(name, pose.header, new_pose.pose)

        orientation = pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        neighbor_facing = self.get_angle_quad(orientation_list)


        # check if all have been found
        if not self._neighbors_started and len(self.neighbor_poses) == len(self.neighbor_position_sub_):
            self._neighbors_started = True
            self.get_logger().info(f"{self.my_name}: All Neighbor Topics Recieved")

        # check to see that all robots are in the right orientation
        if not self.robot_moving:
            test_angle = self.start_heading
            if test_angle == 0 or test_angle == np.pi * 2:
                test_angle = (test_angle + np.pi) % (np.pi * 2)
                neighbor_facing = (neighbor_facing + np.pi) % (np.pi * 2)
            if np.abs(neighbor_facing - test_angle) < self._angle_tolerance:
                self._neighbors_ready[name] = True
            
            if self.desired_heading and not self._robot_moving_wait:
                all_good = True
                for key, value in self._neighbors_ready.items():
                    if not value:
                        all_good = False
                        break
                
                if all_good:
                    self._robot_moving_wait = True
                    self._robot_moving_time = datetime.datetime.now()
                    self.get_logger().info(f"{self.my_name} Sees all neighbors are ready.")

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

         """
            Main logic to compute the Tukey median contour.
            mode 0: Self Distrust.
            mode 1: Normal.
            mode 2: Self Trust.
        """
        '''
        
        X = np.zeros((len(self.neighbor_position)+1,2))
        i = 1
        X[0] = self.position
        for name, neighbor in self.neighbor_position.items():
            X[i] = np.array(neighbor) 
            # print("X[",i,"]: ",X[i])
            i = i+1
        Bx = self.getImprecisionRegions(X,self.imprecision)

        if self.safe_point_mode == 0 or self.safe_point_mode == 1:
            centerpoint = True
            if self.safe_point_mode:
                centerpoint = False
            # for neighbor in self.neighbor_poses:
            #    X[i] = np.array((self.neighbor_poses[neighbor].pose.position.x, self.neighbor_poses[neighbor].pose.position.y))
            
            tc = TukeyContour(X, 0, centerpoint=centerpoint, mode=self.self_trust)

            '''
            This is finding the mean of the tukey median. This is the mean of the vertices of the deepest area in the set.
            '''
            if tc.median_contour.shape[0] > 0:
                # Target is the centroid of the median contour
                self._safe_area = tc.median_contour.tolist()
                self._tukey_depth = float(tc.max_depth)
                self._center_depth = float(tc.center_depth)
                
                safepoint = np.mean(tc.median_contour, axis=0)
                # self.get_logger().info(f"{self.my_name} Has a valid target: {safepoint}")
            else:
                self._safe_area = []
                self._tukey_depth = 0
                self._center_depth = 0
                safepoint = self.position
                self.get_logger().info(f"{self.my_name} Does not have valid target.")
                self.get_logger().info(f"{tc.median_contour} ")
            target = safepoint
           

        elif self.safe_point_mode == 2:
            sp = SafePoint()
            centroid, region, tukey_depth, centerpoint_depth = sp.CPIH_Fast_Safepoint(Bx, 0, self.position, mode=self.self_trust)
            self._tukey_depth = float(tukey_depth)
            self._center_depth = float(centerpoint_depth)

            target = self.position
            if type(centroid) != type(None):
                target = np.array([centroid.x, centroid.y])

            if  region is None or region.is_empty:
                self._safe_area = []
            else:
                if region.geom_type == "MultiPolygon":
                    region = max(region.geoms, key=lambda g: g.area)
                # If not a polygon, skip
                if region.geom_type != "Polygon":
                    self._safe_area = []
                else:
                    self._safe_area = np.array(region.exterior.coords).tolist()
                
        else:
            target = self.position
            self.get_logger().warning(f"{self.my_name}: Invalid mode set. Mode = {self.safe_point_mode}")
            self._safe_area = []

        if (np.linalg.norm(self.position-target)<0.3):
            self.complete = True

        # pdb.set_trace()
        self.move_to_position(target)


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=4

    ## python3 CPIH.py -i 1 -n 1 2 3 -s --ros-args -p robot.neighborhood_mode:=global -p robot.neighborhood_global:=[1,1,0,1,0,1,0,1,1] -p robot.neighborhood_size:=3
    ## ros2 run agent_control CPIH.py --ros-args --params-file src/agent_control/config/CPIH/Network1.yaml -p robot.id:=1 -p robot.neighbors:=['1','2','3', '4'] -p logging.enabled:=true -p mode.sim:=true 
    ## ros2 run agent_control CPIH.py --ros-args --params-file src/agent_control/config/CPIH/BaseConfig.yaml --params-file src/agent_control/config/CPIH/AdvesaryOne.yaml -p robot.id:=3 -p robot.neighbors:='[1,3,4,5,6,7,11,12,16]'


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
