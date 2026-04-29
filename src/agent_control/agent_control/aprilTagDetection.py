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
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from irobot_create_msgs.srv import ResetPose
from std_srvs.srv import Trigger

from dotmap import DotMap

import pdb


class AprilTagDetectorNode(Agent):

    def __init__(self, node_name, reset_odom=False):
        self.extra_param_update_map = {
            "tag.follow": "follow_tag",
            "tag.verbose": "tag_verbvose"
        }
        super().__init__(node_name)

        self.declare_parameter("tag.follow", False) 
        self.follow_tag = self.get_parameter("tag.follow").value

        self.declare_parameter("tag.verbose", False)
        self.tag_verbvose = self.get_parameter("tag.verbose").value

        # self._min_angle = 0.1

        odom_topic = f"/{self.my_name}/odom"
        image_topic = f"/{self.my_name}/oakd/rgb/preview/image_raw"
        camera_topic = f"/{self.my_name}/oakd/rgb/preview/camera_info"
        self._camera_started = False
        self._camera_setup = False
        self.fx, self.cx, self.fy, self.cy = None, None, None, None 

        # Setting Up Odometry information
        self.odom_subscrition = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        '''
        Will set current position to [0,0,0] and orientation to 0 Radians. 
        Due to how the system is setup, this should be facing the back wall and will actuall read at pi to match mocap

        Can set this to a default value using this format before doing the call
        ## req.x = 1.0
        ## req.y = 1.0
        ## req.z = 0.0
        ## req.theta = 0.0
        '''
        # Reseting the Pose
        if reset_odom:
            self.reset_pose_client = self.create_client(ResetPose, f"/{self.my_name}/reset_pose")
            self.reset_pose_client.wait_for_service()
            # Make Request
            req = ResetPose.Request()
            future = self.reset_pose_client.call_async(req)
            future.add_done_callback(self.pose_reset_done)

        ## Starting and Stopping Camera
        self.start_camera_client = self.create_client(Trigger, f"/{self.my_name}/oakd/start_camera")
        self.stop_camera_client = self.create_client(Trigger, f"/{self.my_name}/oakd/stop_camera")
        self.start_camera_client.wait_for_service() # Waiting for services to be available
        self.stop_camera_client.wait_for_service() # Waiting for services to be available

        if not self._camera_setup:
            # Start Camera
            start_req = Trigger.Request()
            start_future = self.start_camera_client.call_async(start_req)
            start_future.add_done_callback(self.after_start)
            '''
            ros2 service call /robot1/oakd/start_camera std_srvs/srv/Trigger "{}"
            '''

            # # Stop Camera
            # stop_req = Trigger.Request()
            # stop_future = self.stop_camera_client.call_async(stop_req)
            # stop_future.add_done_callback(self.after_stop)
            '''
            ros2 service call /robot1/oakd/stop_camera std_srvs/srv/Trigger "{}"
            '''

        # Setting Up Image information
        self.bridge = CvBridge()

        self.img_subscription = self.create_subscription(
            Image,
            image_topic, 
            self.image_callback,
            10)

        self.camera_subscription = self.create_subscription(
            CameraInfo,
            camera_topic,
            self.setup_camera_callback,
            10
        )


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
        self.get_logger().info(f"{self.my_name} AprilTag Detector Node initialized and subscribed to {image_topic}")
        self.display_image = False
        # self._circle_angles = [0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 5*np.pi/4, 3*np.pi/2, 7*np.pi/4]
        self._circle_angles = np.linspace(0, 2*np.pi, 16)
        self._circle_index = 0
        self._circle_counter = 0
        self._circle_counter_max = 10
        self._finding_neighbor_vision = False
        self._test = False
        self._test_counter = 0

    def odom_callback(self, msg: Odometry):
        self.update_position_(msg.header, msg.pose.pose, False)

        # robot_position = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation

        # x, y, z = robot_position.x, robot_position.y, robot_position.z 
        # qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

        # self.position = self.correct_position_(x,y, orientation)
        # self.direction_heading = self.get_angle_quad(orientation)

        # if self.count % 60 == 0:
        #     self.get_logger().info(f"Pose (x,y,z): {x:.2f}, {y:.2f}, {z:.2f} meters")
        #     self.get_logger().info(f"Direction Heading: {self.direction_heading}")
        
        # self.count = (self.count + 1) % 60

    def pose_reset_done(self, future):
        result = future.result()
        self.get_logger().info(f"{self.my_name} Odomentry has been reset.")
    
    def after_start(self, future):
        result = future.result()    # result.success, result.message
        if result.success:
            self._camera_started = True
            self.get_logger().info(f"{self.my_name} Camera Started")
        else:
            self.get_logger().warning(f"{self.my_name} Failed to start the camera!!")

    def after_stop(self, future):
        result = future.result()
        self.get_logger().info(f"Stop Camera Result: {result.success}, message: '{result.message}'")

    def setup_camera_callback(self, msg: CameraInfo):
        # Get topic /robot1/oakd/rgb/preview/camera_info
        '''
        header:
        stamp:
            sec: 1724534603
            nanosec: 781594645
        frame_id: oakd_rgb_camera_optical_frame
        height: 250
        width: 250
        distortion_model: rational_polynomial
        d:
        - -4.860828876495361
        - 17.2819766998291
        - 0.00020376102474983782
        - 0.0010251018684357405
        - -25.172250747680664
        - -4.943647384643555
        - 17.51442527770996
        - -25.290760040283203
        k:
        - 196.50531005859375
        - 0.0
        - 126.86713409423828
        - 0.0
        - 196.50531005859375
        - 125.84786987304688
        - 0.0
        - 0.0
        - 1.0
        r:
        - 1.0
        - 0.0
        - 0.0
        - 0.0
        - 1.0
        - 0.0
        - 0.0
        - 0.0
        - 1.0
        p:
        - 196.50531005859375
        - 0.0
        - 126.86713409423828
        - 0.0
        - 0.0
        - 196.50531005859375
        - 125.84786987304688
        - 0.0
        - 0.0
        - 0.0
        - 1.0
        - 0.0
        binning_x: 0
        binning_y: 0
        roi:
        x_offset: 0
        y_offset: 0
        height: 0
        width: 0
        do_rectify: false
        ---
        '''
        # d: [k1, k2, p1, p2, k3, k4, k5, k6]
        # k: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        if self._camera_started and not self._camera_setup:
            self.fx, self.cx, self.fy, self.cy = msg.k[0], msg.k[2], msg.k[4], msg.k[5]
            self._camera_distortion = np.array(msg.d, dtype=np.float32)
            self._camera_k = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self._camera_setup = True
            self.get_logger().info(f"{self.my_name} Camera Info Recieved and parameters setup")

    def image_callback(self, msg):
        if self._camera_setup:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'CvBridge Error: {e}')
                return

            # Convert the color image to grayscale for AprilTag detection
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # camera_params = [1486.56, 1489.024, 953.16, 560.76] # Example RGB params
            camera_params = [
                self.fx,    # fx
                self.fy,    # fy
                self.cx,    # cx
                self.cy     # cy
            ]
            tag_size = 0.200 # Meters
            x_ratio = 1.36655

            tags = self.at_detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=tag_size
            )

            if self.display_image:
                debug_image = self.draw_tags(cv_image, tags)
                
                # Display the frame with detections
                cv2.imshow("AprilTag Detections", debug_image)
                cv2.waitKey(1)

            # loop through the tags
            if self.follow_tag:
                self.example_follow_tag(tags)

            for tag in tags:
                tx, ty, tz = tag.pose_t.flatten()              # [ Right is postive , Down is Postive, Positive Forward]
                x, y, z = tz*x_ratio, -tx, -ty                   # On Robot 3, the z needed offset 58.9% to be accurate. Left and right was good
                # Check determinant
                r_tag = tag.pose_R
                if np.linalg.det(r_tag) < 0:
                    r_tag = -r_tag  # Flip axes to make it right-handed
                quat = R.from_matrix(r_tag).as_quat() # [x, y, z, w]

                # finding tag position
                camera_offset = np.array([0,0,0])
                robot_position = [self.position[0], self.position[1], 0]
                robot_wf = R.from_quat(self.quaternion) # rotate robot frame to wold frame
                camera_wf = robot_position + robot_wf.apply(camera_offset)
                neighbor_wf = robot_position + robot_wf.apply([x,y,z])

                # finding tag orientation
                neighbor_rotation = R.from_quat(quat)
                neighbor_rotation_wf = robot_wf * neighbor_rotation
                neighbor_quat_wf = neighbor_rotation_wf.as_quat()

                pose = DotMap({
                    "position": {
                    "x": neighbor_wf[0],
                    "y": neighbor_wf[1],
                    "z": neighbor_wf[2]
                    },
                    "orientation": {
                        "x": neighbor_quat_wf[0],
                        "y": neighbor_quat_wf[1],
                        "z": neighbor_quat_wf[2],
                        "w": neighbor_quat_wf[3]
                    }
                })
            
                self.update_neighbor_position_(tag.tag_id, msg.header, pose)
                # self.update_neighbor_position_("00", msg.header, pose1)
                # self.get_logger().info(f"Tag Detected: {tag.tag_id}")
        
    def example_follow_tag(self, tags=None):
        move_x = 0.0
        move_z = 0.0
        if tags:
            # Process and log pose data
            x, y, z = tags[0].pose_t.flatten()              # [ Right is postive , Down is Postive, Positive Forward]
            quat = R.from_matrix(tags[0].pose_R).as_quat() # [x, y, z, w]
            # Simple angle calculation (check your coordinate system definition)
            angle = np.remainder((np.arctan2(2 * (quat[3] * quat[2] + quat[0] * quat[1]), 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])) + np.pi), 2 * np.pi)

            # # if self.count % 60 == 0:
            # #     self.get_logger().info(f"Tag ID: {tags[0].tag_id} | Pose (x,y,z): {x:.2f}, {y:.2f}, {z:.2f} meters")
            # #     self.get_logger().info(f"Angle: {angle:.2f} radians")
            
            # # self.count = (self.count + 1) % 60
            # self.get_logger().info(f"Tag ID: {tags[0].tag_id} | Pose (x,y,z): {x:.2f}, {y:.2f}, {z:.2f} meters")
            # self.get_logger().info(f"Angle: {angle:.2f} radians")

            # Move forward and backward
            if tags[0].tag_id == 0:
                if z > 1:
                    move_x = 0.2
                elif z < 0.5:
                    move_x = -0.1

            elif tags[0].tag_id == 1:
                if z > 2:
                    move_x = 0.2
                elif z < 1:
                    move_x = -0.1

            else:
                if z > 0.2:
                    move_x = 0.2
                elif z < 0.05:
                    move_x = -0.1

            # #Turn left and right
            if x > 0.05:
                move_z = -0.5
            elif x < -0.05:
                move_z = 0.5


            if self.tag_verbvose:
                print(f"TagID: {tags[0].tag_id}")
                print(f"Distance: {z}, Side: {x}")
                print(f"Move: {move_x}, {move_z}")

        
        self.move_robot_(move_x, move_z)

    def draw_tags(self, image, tags):
        for tag in tags:
            tag_family = tag.tag_family
            tag_id = tag.tag_id
            center = tag.center
            corners = tag.corners

            center = (int(center[0]), int(center[1]))
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))

            
            cv2.circle(image, (center[0], center[1]), 5, (255, 0, 255), 2)

            cv2.line(image, (corner_01[0], corner_01[1]), (corner_02[0], corner_02[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_02[0], corner_02[1]), (corner_03[0], corner_03[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_03[0], corner_03[1]), (corner_04[0], corner_04[1]), (0, 255, 0), 2)
            cv2.line(image, (corner_04[0], corner_04[1]), (corner_01[0], corner_01[1]), (0, 255, 0), 2)
            cv2.putText(image, str(tag.tag_id), (center[0] - 50, center[1] - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
        return image

    def find_neighbors_vision(self):
        if not self._finding_neighbor_vision:
            self._finding_neighbor_vision = True

        if not self.desired_heading:
            if self._circle_index >= len(self._circle_angles):
                if self._circle_counter == 0:
                    self._finding_neighbor_vision = False
                    self._circle_index = 0
                    return False
            else:
                self.move_to_angle(self._circle_angles[self._circle_index])
        else:
            if self._circle_counter > self._circle_counter_max:
                self.desired_heading = False
                self._circle_index += 1
                self._circle_counter = 0
            else:
                self._circle_counter += 1
            
        return True

    def controller(self):

        # # #  2 tiles away is about: 1.1402568817138672
        # # # Max Read about 11 Squares: 6.14m
        # if len(self.neighbor_poses):
        #     if self._test_counter > 10 and self._test_counter < 100:
        #         self.get_logger().info(f"{self.neighbor_poses}")
        #         self.get_logger().info(f"{self._laser_scan[int(len(self._laser_scan)/2)]}")
        #         self._test_counter = 0
        #     else:
        #         self._test_counter += 1

        '''
        2 tile away = ~ 1.14
        166x168 Tag 
            => Tag Size = 0.1651 x_ratio = 1.6978
        90 Degree Rotated = x: 1.212
        Correct Rotation = x: 1.15
            => Tag Size = 0.168 x_ratio = 1.6978
        90 Degree = x:1.18
        Correct Rotation = X:1.18

        200x168 Tag
            => Tag Size = 0.168 x_ratio = 1.6978
        X:1.165
            => Tag Size = 0.200 x_ratio = 1.6978
            Range => 11 Tiles (7.181)
        X:1.395

        82x82.5 Tag
            => Tag Size = 0.082 x_ratio = 1.6978
            Range => 5.5 Tiles (2.98)
        X:1.149

        40x40 Tag
            => Tag Size = 0.0.04 x_ratio = 1.6978
            Range => 2.5 Tiles (1.35)
        X:1.146
        

        * self._laser_scan[int(len(self._laser_scan)/2 at 2 tiles is 5.264 (turtlebot3) AprilTag distance: 1.103
        * self._laser_scan[int(len(self._laser_scan)/2 at 2 tiles is 5.768 (turtlebot1) AprilTag distance: 1.12


        '''

        self.set_led_mode_("FINISHED")

        if not self.follow_tag:
            ### Vision Concenus####
            # 'x': -0.644445846281644, 'y': 0.11759494681753993, 'z': 0.033320860385021155
            # self.move_to_position([1.14, 0.0])
            # self.move_to_position([1.6664, 0.5419])
            # self.move_to_position([0.0,0.0])

            if not self._test:
                if not self.find_neighbors_vision():
                    self._test = True
            else:
                total = 0
                not_too_close = 1
                distances = np.array([])

                for name, neighbor in self.neighbor_position.items():
                    difference = (np.array(neighbor) - np.array(self.position))/2
                    distances = np.append(distances, np.linalg.norm(difference))
                    if np.linalg.norm(difference) > not_too_close:
                        weight = 1
                    else:
                        weight = 0
                    total += weight * difference
                

                self.move_direction(total)

                if self.destination_reached:
                    self._test = False
                    self.get_logger().info(f"Completed Movement. Checking Neighbors")

        return
                
    
    def end_controller(self):
        return

def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=1

    # need numpy under 2.0
    # pip install numpy==1.26.4
    # Notes SCIPY needs < 1.25.0
    # pip install numpy==1.24.4

    # --ros-args -p robot.id:=1 -p robot.neighbors:="[1,2,3]" -p laser.avoid:=false -p logging.enabled:=false
    
    apriltag_detector_node = None
    try:
        rclpy.init(args=args)
        apriltag_detector_node = AprilTagDetectorNode("AprilTagDetector")
        rclpy.spin(apriltag_detector_node)
    except KeyboardInterrupt:
        traceback.print_exc()
    finally:
        if apriltag_detector_node:
            apriltag_detector_node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
