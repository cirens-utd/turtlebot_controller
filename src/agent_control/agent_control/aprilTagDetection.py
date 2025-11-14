#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
import traceback

from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pdb


class AprilTagDetectorNode(Agent):

    # def __init__(self, my_number, my_neighbors=[], *args, sim=False, sync_move=False,
    #     destination_tolerance=0.01,logging=False,
    #     restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
    #     laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
    #     neighbor_avoid=True, neighbor_delay=5):
    #     super().__init__(my_number, my_neighbors, sim=sim, sync_move=sync_move, 
    #                     destination_tolerance=destination_tolerance, logging=logging,
    #                     restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
    #                     laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
    #                     neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)


    def __init__(self):
        super().__init__('apriltag_detector_node')
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        # Change '/camera/image_raw' to the actual topic name you are publishing
        self.subscription = self.create_subscription(
            Image,
            '/crobot1/oakd/rgb/preview/image_raw', 
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize the AprilTag detector
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)
        
        self.count = 0
        self.get_logger().info("AprilTag Detector Node initialized and subscribed to /camera/image_raw")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Convert the color image to grayscale for AprilTag detection
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # --- Detection Logic ---
        # Note: You need the correct camera parameters for pose estimation to work accurately.
        # The parameters below were from your original 'robot1' comment. 
        # Make sure these match your *actual* ROS camera calibration parameters.
        camera_params = ([1486.56, 1489.024, 953.16, 560.76]) # Example RGB params
        tag_size = 0.05 # Meters

        tags = self.at_detector.detect(
            gray_image,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size,
        )

        debug_image = self.draw_tags(cv_image, tags)
        
        # Display the frame with detections
        # cv2.imshow("AprilTag Detections", debug_image)
        # cv2.waitKey(1)

        if tags:
            # Process and log pose data
            x, y, z = tags[0].pose_t.flatten()
            quat = R.from_matrix(tags[0].pose_R).as_quat() # [x, y, z, w]
            # Simple angle calculation (check your coordinate system definition)
            angle = np.remainder((np.arctan2(2 * (quat[3] * quat[2] + quat[0] * quat[1]), 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])) + np.pi), 2 * np.pi)

            if self.count % 60 == 0:
                self.get_logger().info(f"Tag ID: {tags[0].tag_id} | Pose (x,y,z): {x:.2f}, {y:.2f}, {z:.2f} meters")
                self.get_logger().info(f"Angle: {angle:.2f} radians")
            
            self.count = (self.count + 1) % 60

    def draw_tags(self, image, tags):
        for tag in tags:
            center = tag.center
            corners = tag.corners

            center = (int(center[0]), int(center[1]))
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))

            # 中心
            cv2.circle(image, (center[0], center[1]), 5, (255, 0, 255), 2)

            # 各辺
            cv2.line(image, (corner_01[0], corner_01[1]), (corner_02[0], corner_02[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_02[0], corner_02[1]), (corner_03[0], corner_03[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_03[0], corner_03[1]), (0, 255, 0), 2)
            cv2.line(image, (corner_04[0], corner_04[1]), (0, 255, 0), 2)
            cv2.putText(image, str(tag.tag_id), (center[0] - 50, center[1] - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        return image


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=1

    # need numpy under 2.0
    # pip install numpy==1.26.4
    # Notes SCIPY needs < 1.25.0
    # pip install numpy==1.24.4
    
    rclpy.init(args=args)
    apriltag_detector_node = AprilTagDetectorNode()
    try:
        rclpy.spin(apriltag_detector_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    apriltag_detector_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
