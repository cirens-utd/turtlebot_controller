import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import argparse
import datetime

import pdb

class Agent(Node):
    def __init__(self, my_number, my_neighbors=[], *args, 
        position_rate=2, laser_avoid=True, laser_distance=0.4, laser_delay=5, laser_walk_around=0):
        # start with this agents number and the numbers for its neighbors
        name = f"robot{my_number}"
        super().__init__(name)
        self.my_name = name
        self.my_number = my_number
        self.test_index = 0

        # Create Publisher for movement
        self.cmd_vel_pub_ = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
        
        # Create Subscriber for position
        self.position_sub_ = self.create_subscription(PoseArray, f"/vrpn_mocap/turtlebot{self.my_number}/pose", self.pose_callback, 10)
        self.neighbor_position_sub_ = {}
        for number in my_neighbors:
            try:
                self.neighbor_position_sub_[number] = self.create_subscription(
                    PoseArray, 
                    f"/vrpn_mocap/turtlebot{number}/pose", 
                    lambda msg,name=number: self.neighbor_pose_callback(msg, name), 
                    10
                )
            except:
                print(f"Could not subscribe to turtlebot{number} Position")
    
        self.lidar_sub_ = self.create_subscription(LaserScan, f"/{name}/scan", self.lidar_callback, 10)
        # track my location
        self._position = None

        # track where my neighbors are
        self._neighbor_position = {}
        self.get_logger().info(f"Robot{my_number} has been started.")

        # Avoidance Vars
        self.laser_avoid = laser_avoid          # Boolean to use laser to avoid obstructions
        self.laser_distance = laser_distance    # Minimum distance you can get to an object
        self._laser_range_setup = False         # Boolean to setup laser indexs 
        self._laser_rf_index = 285              # Index value of where the robot will colide with something in front of it on the right side (-0.35 Radians)
        self._laser_lf_index = 353              # Index value of where the robot will colide with something in front of it on the left side (0.35 Radians)
        self._laser_right_index = 160           # Index value of where an object is directly to the right of the robot (-pi/2 Radians)
        self._laser_forward_index = 320         # Index value of where an object is directy in front of the robot (0 Radians)
        self._laser_left_index = 480            # Index value of where an object is directly to the right of the robot (pi/w Raidians)
        self._laser_obstructed_forward = False  # Boolean to know we are clear in front of us
        self._laser_obstructed_right = False    # Boolean to know we are clear on the right
        self._laser_obstructed_left = False     # Boolean to know we are clear on the left
        self._laser_walk_around = laser_walk_around # If 0 robot will go left and 1 will go right
        self._laser_delay = laser_delay         # Number of seconds before avoidance is taken 
        self._laser_scan = None
        self._laser_min_angle = None
        self._laser_max_angle = None
        self._laser_angle_increment = None

        # status for the agent
        self._position_refresh_rate = position_rate
        self._motion_complete = False
        self._direction_facing = 0
        self._path_obstructed_time = None
        self._path_obstructed = False
        self._path_obstructed_laser = False

    def pose_callback(self, pose: PoseArray):
        orientation = pose.poses[0].orientation
        x,y = pose.poses[0].position.x, pose.poses[0].position.y
        self.position = [x,y]
        self.direction_facing = self.get_angle_quad(orientation)
        self.controller()

    def neighbor_pose_callback(self, pose: PoseArray, name):
        x,y = pose.poses[0].position.x, pose.poses[0].position.y
        self._neighbor_position[name] = [x,y]
    
    def lidar_callback(self, msg: LaserScan):

        # Something is in the way if there is something between 2.8 and 3.469 radian
        # if not self._laser_range_setup:
        #     self.setup_laser_config_(msg)

        # In front of bot at 0.4m away
        # 285 - 353
        # 2.800 - 3.469 - angle steps
        # -0.342 - 0.327 - radians

        # Behind bot at 0.4m away
        # 0 - 33; 605 - 639
        # 0 - 0.324; 5.95 - 6.28
        # -3.14 - -2.816; 2.81 - 3.14

        # print("Calling Lidar Scan")
        # # ** These were never used???
        # for index, value in enumerate(msg.intensities):
        #     if value != 0:
        #         print(f"Intensity Index: {index}, Value: {value}")

        # for index, value in enumerate(msg.ranges):
        #     if value != np.inf:
        #         print(f"Range Index: {index}, Value: {value}")

        self._laser_scan = np.array(msg.ranges)
        straight_ahead_range = np.array(msg.ranges[self._laser_rf_index:self._laser_lf_index])
        right_side_range = np.array(msg.ranges[self._laser_right_index:self._laser_forward_index])
        left_side_range = np.array(msg.ranges[self._laser_left_index:self._laser_forward_index])

        # check if path straight ahead is blocked
        if self.laser_avoid and (straight_ahead_range < self.laser_distance).any():
            self.path_obstructed_laser = True
            # turned to False in movement method
            self._laser_obstructed_forward = True
        else:
            self._laser_obstructed_forward = False

        if (right_side_range <= self.laser_distance + 0.1).any():
            self._laser_obstructed_right = True
        else:
            self._laser_obstructed_right = False
        
        if (left_side_range <= self.laser_distance + 0.1).any():
            self._laser_obstructed_left = True
        else:
            self._laser_obstructed_left = False

        # Example of walking around using laser
        """
        # Run walk around (turn left)
        test_range = np.array()
        if (test_range < 0.5).any():
            # Rotate CCW until we can move again
            self.move_robot_(0.0, 1.0)

        # 160 is when the obstruction is on our right side
        elif (np.array(msg.ranges[160:320]) < 0.6).any():
            self.move_robot_(1.0, 0.0)
        else:
            self.move_robot_(0.0, -1.0)
        """
        """
        if (test_range < 0.5).any():
            # Rotate CW until we can move again
            self.move_robot_(0.0, -1.0)

        # 480 is when the obstruction is on our left side
        elif (np.array(msg.ranges[320:480]) < 0.6).any():
            self.move_robot_(1.0, 0.0)
        else:
            self.move_robot_(0.0, 1.0)
        """


    @property
    def motion_complete(self):
        return self._motion_complete
    @motion_complete.setter
    def motion_complete(self, value):
        self._motion_complete = bool(value)

    @property
    def direction_facing(self):
        return self._direction_facing
    @direction_facing.setter
    def direction_facing(self, q):
        self._direction_facing = q

    @property
    def position(self):
        return self._position
    @position.setter
    def position(self, value):
        # Value = [x,y]
        self._position = np.array(value) 

    @property
    def path_obstructed_laser(self):
        return self._path_obstructed_laser
    @path_obstructed_laser.setter
    def path_obstructed_laser(self, value):
        self._path_obstructed_laser = bool(value)
        self.set_path_obstructed_()

    @property
    def path_obstructed(self):
        return self._path_obstructed

    @property
    def rate(self):
        return self._position_refresh_rate
    
    def set_path_obstructed_(self):
        """
        Check the obsturction vars and if any are true, set _path_obstructed to True
        """
        values = [self.path_obstructed_laser]
        cur_value = any(values)
        if not self._path_obstructed and cur_value:
            self._path_obstructed_time = datetime.datetime.now()
        elif self._path_obstructed and not cur_value:
            self._path_obstructed_time = None

        self._path_obstructed = cur_value

    def get_angle_quad(self,q):
        """
        Finds the heading angle of the robot from its quaternion orientation.
        :param q: quaternion rotation from ROS msg
        :return: radians 
        """
        return np.remainder((np.arctan2(2 * (q.w * q.z + q.x * q.y),1 - 2 * (q.y * q.y + q.z * q.z)) + np.pi) , 2 * np.pi)

    def angle(self, x,y):
        """
        Finds the angle between x and y
        :param x: an array of [x, y] position for x
        :param y: an array of [x, y] position for y
        :return: radians
        """
        x = np.array(x)
        y = np.array(y)
        diff = y - x 
        slope = diff[1]/diff[0]
        angle = np.arctan2(diff[1], diff[0])
        return np.remainder(angle + np.pi, 2 * np.pi)

    def diff_angles(self, x, y):
        """
        Finds the smalles angle between two angles.
        :param x: a radian between 0 - 2pi
        :param y: a radian between 0 - 2pi
        :return: radians
        """
        diff = x - y 
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        return diff

    def setup_laser_config_(self, msg):
        """
        Sets Laser Configuration
        :param msg: Ros MSG for LaserScan
        :return: None
        """
        # print(f"Header: {msg.header}")
        " std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=594, nanosec=800000000), frame_id='base_link') "
        # print(f"Angle Min: {msg.angle_min}")
        " -3.1400000104904175 "
        # print(f"Angle Max: {msg.angle_max}")
        " 3.1400000104904175 "
        # print(f"Angle Increment: {msg.angle_increment}")
        " 0.009827855974435806 "
        # print(f"Time Increment: {msg.time_increment}")
        " 0.0 "
        # print(f"Scan Time: {msg.scan_time}")
        " 0.0 "
        # print(f"Range Min: {msg.range_min}")
        " 0.300000 "
        # print(f"Ranges: {len(msg.ranges)}")
        " 640 "
        # print(f"Intesities: {len(msg.intensities)}")
        " 640 "

        """
        # front of it on the right side (2.8 Radians)
        # front of it on the left side (3.5 Radians)
        # directly to the right of the robot (-pi/2 Radians)
        # directy in front of the robot (0 Radians)
        # directly to the right of the robot (pi/w Raidians)
        """        

        min_angle = msg.angle_min
        max_angle = msg.angle_max
        angle_increment = msg.angle_increment

        self._laser_min_angle = min_angle
        self._laser_max_angle = max_angle
        self._laser_angle_increment = angle_increment

        rf_radian = -0.35
        lf_radian = 0.35
        r_radian = -np.pi/2
        l_radian = np.pi
        f_radian = 0

        rf_index = self.laser_radian_index_(rf_radian, True)
        lf_index = self.laser_radian_index_(lf_radian, False)
        r_index = self.laser_radian_index_(r_radian, True)
        l_index = self.laser_radian_index_(l_radian, False)
        f_index = self.laser_radian_index_(f_radian, True)

        self._laser_rf_index = rf_index              
        self._laser_lf_index = lf_index              
        self._laser_right_index = r_index           
        self._laser_forward_index = f_index         
        self._laser_left_index = l_index 
        self._laser_range_setup = True

    def laser_radian_index_(self, angle, floor=True):
        """
        Function will find the index of the given radian.
        :param angle: radian you want to find
        :param floor: optional argument. When set to False, you will get the ceiling funciton instead
        """
        if floor:
            return np.floor((angle - self._laser_min_angle) / self._laser_angle_increment)
        else:
            return np.ceil((angle - self._laser_min_angle) / self._laser_angle_increment)

    def path_clear_laser_(self, desired_location, angle, magnitude, tolerance=2):
        """
        This function will check to see if the laser shows any more obstructions between us and our goal
        :param desired_location: an array of the position we want to get to
        :param angle: the angle between us and the goal
        :param magnitude: the magnitude between us and our goal
        :param tolerance: option parameter for the cutoff point for the clearance
        :return boolean: returns True if our path is clear and false if it is not
        """

        self._laser_min_angle = min_angle
        self._laser_max_angle = max_angle
        self._laser_angle_increment = angle_increment

        diff_angle = self.diff_angles(angle, self.direction_facing)
        idx = self.laser_radian_index_(angle, True)
        return self._laser_scan[idx] > tolerance

    def scale_movement_(self, value):
        '''
        Creating cut off points for the movement
        '''
        new_value = max(-2.0, min(value, 2.0))
        if np.abs(new_value) > 0.01 and np.abs(new_value) < 0.5:
            new_value = 0.5 * (new_value / np.abs(new_value))
        elif np.abs(new_value) <= 0.01:
            new_value = 0.0
        return new_value

    def move_to_position(self, desired_location):
        """
        Start moving the robot to the desired direction. This handles when to drive vs when to turn. Also handles setting speeds for each
        :param desired_location: a position vector [x, y] you want to go to
        :return: None
        """

        desired_location = np.array(desired_location)
        magnitude = np.linalg.norm(self.position - desired_location)
        angle = self.angle(self.position, desired_location)
        
        if not self.path_obstructed:
            z = self.scale_movement_(self.diff_angles(angle, self.direction_facing))
            x = self.scale_movement_(magnitude)
            # x = 0.0
            # print(f"{angle} , {self.direction_facing}, {z}")
            # print(x, z, magnitude, angle, self.direction_facing)

            # # check to make sure we are not facing the wrong direction
            # if np.abs(z) <= 3*np.pi/2:
            #     self.move_robot_(x, z)
            # # going the oposite direction
            # else:
            #     self.move_robot_(-x, -z)

            if magnitude > 1:
                if np.abs(z) < 3*np.pi/2:
                    if np.abs(z) > 1:
                        self.move_robot_(x/4, z)
                    else:
                        self.move_robot_(x, z)
                else:
                    self.move_robot_(0.0, z)
            else:
                if np.abs(z) > 0.001:
                    self.move_robot_(0.0, z)
                else:
                    self.move_robot_(x, z)
        elif self.laser_avoid and self.path_obstructed_laser:
            self.move_around_laser_(desired_location)
        else:
            self.move_robot_(0.0, 0.0)
            self.get_logger().info(f"Robot{my_number} is obstructed but no detour method selected.")

    def move_robot_(self, x, z):
        """
        Internal Method used to send ROS topic
        :param x: The desired value to put into linear X
        :param z: The desired value to put into angluar Z
        :return: None
        """
        cmd = Twist()
        cmd.linear.x = x 
        cmd.angular.z = z 
        self.cmd_vel_pub_.publish(cmd)

    def move_around_laser_(self, desired_location):
        """
        Internal Method to get around an object using the laser. Will wait a set period of time before acting.
        If _laser_walk_around = 0, the robot will go left and if 1, it will go right
        :param desired_location: Array of desired location
        :return: None
        """

        desired_location = np.array(desired_location)
        magnitude = np.linalg.norm(self.position - desired_location)
        angle = self.angle(self.position, desired_location)

        if self._path_obstructed_time + datetime.timedelta(seconds=self._laser_delay) <= datetime.datetime.now():
            # go right
            if self._laser_walk_around:
                # Rotate CW until we can move again
                if self._laser_obstructed_forward:
                    self.move_robot_(0.0, -1.0)
                # Drive Forward until clear
                elif self._laser_obstructed_left:
                    self.move_robot_(1.0, 0.0)
                # turn back forward
                else:
                    self.move_robot_(0.0, 1.0)
            # go left
            else:
                # Rotate CCW until we can move again
                if self._laser_obstructed_forward:
                    self.move_robot_(0.0, 1.0)
                # Drive Forward until clear
                elif self._laser_obstructed_right:
                    self.move_robot_(1.0, 0.0)
                # turn back forward
                else:
                    self.move_robot_(0.0, -1.0)
        else:
            # waiting to see if obsturction moves
            self.move_robot_(0.0, 0.0)
        
        # Check to see if path is still blocked
        if not self._laser_obstructed_right and not self._laser_obstructed_forward:
            self.path_obstructed_laser = not self.path_clear_laser_(desired_location, angle, magnitude)

    def controller(self):
        """
        This is the main logic for controlling the agent. 

        This should be overridden by the derived classes, and the default
        raises NotImplementedError.

        You should find your desired position you want this agent to go to and then send that to the self.move_to_position method

        :raises: NotImplementedError
        """
        # # find your desired location (x, y) and pass it into self.move_to_position((x,y))
        # positions = [[29,3],[10,0], [-10,10], [5,2]]
        
        # if np.linalg.norm(self.position - positions[self.test_index]) > 0.1:
        #     self.move_to_position(positions[self.test_index])
        # else:
        #     self.test_index += 1
        #     if len(positions) >= self.test_index:
        #         print("moving to next point", positions[self.test_index])
        #     else:
        #         print("done")
        #         self.move_robot_(0.0,0.0)


        self.move_to_position([10,0])
        # self.move_robot_(0.0, 0.0)
        return
        # raise NotImplementedError('perform() not implemented for Substitution base class.')

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--number", default="1")
    scrpt_args = parser.parse_args()

    rclpy.init(args=args)
    my_robot = Agent(int(scrpt_args.number))
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()