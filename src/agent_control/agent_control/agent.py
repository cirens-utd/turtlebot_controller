import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import argparse
import datetime

import pdb

class Agent(Node):
    def __init__(self, my_number, my_neighbors=[], *args, sync_move=False,
        destination_tolerance=0.01,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2,
        neighbor_avoid=True, neighbor_delay=5):
        # start with this agents number and the numbers for its neighbors
        name = f"robot{my_number}"
        super().__init__(name)
        self.my_name = name
        self.my_number = my_number
        self._diameter = 0.4
        self.test_index = 0

        self.get_logger().info(f"{self.my_name} has been started.")

        # Create Publisher for movement
        self.cmd_vel_pub_ = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
        
        # Create Subscriber for position
        self.position_sub_ = self.create_subscription(PoseStamped, f"/vrpn_mocap/turtlebot{self.my_number}/pose", self.pose_callback_, 10)
        self.neighbor_position_sub_ = {}
        for number in my_neighbors:
            try:
                self.neighbor_position_sub_[number] = self.create_subscription(
                    PoseStamped, 
                    f"/vrpn_mocap/turtlebot{number}/pose", 
                    lambda msg,name=number: self.neighbor_pose_callback_(msg, name), 
                    10
                )
                self.get_logger().info(f"{self.my_name} Subscribed to neighbor number {number}")
            except:
                self.get_logger().warning(f"Could not subscribe to turtlebot{number} Position")
    
        self.lidar_sub_ = self.create_subscription(LaserScan, f"/{name}/scan", self.lidar_callback_, 10)
        
        # track my location
        self._position = None

        # track where my neighbors are
        self.neighbor_position = {}

        # Laser Avoidance Vars
        self.laser_avoid = laser_avoid          # Boolean to use laser to avoid obstructions
        self.laser_distance = laser_distance    # Minimum distance you can get to an object
        self._laser_range_setup = False         # Boolean to setup laser indexs 
        self.rf_radian = -np.pi/3               # This is used to setup the radian position for laser colision
        self.lf_radian = np.pi/3                 # This is used to setup the radian position for laser colision
        self.r_radian = -np.pi/2                # This is used to setup the radian position for laser colision
        self.l_radian = np.pi/2                 # This is used to setup the radian position for laser colision
        self.f_radian = 0                       # This is used to setup the radian position for laser colision
        self._laser_obstructed_forward = False  # Boolean to know we are clear in front of us
        self._laser_obstructed_right = False    # Boolean to know we are clear on the right
        self._laser_obstructed_left = False     # Boolean to know we are clear on the left
        self._laser_walk_around = laser_walk_around # If 0 robot will go left and 1 will go right and 2 will decide based on laser
        self._laser_delay = laser_delay         # Number of seconds before avoidance is taken 
        self._laser_scan = None                 # Last known value of the laser scanner
        self._laser_min_angle = None            # Min angle of the laser scanner (setup in laser setup)
        self._laser_max_angle = None            # Max angle of the laser scanner (setup in laser setup)
        self._laser_angle_increment = None      # Angle increment (setup in laser setup)
        self._laser_dynamic_left = False        # Flag used to determine if we are going left
        self._laser_dynamic_right = False       # Flag used to determine if we are going right

        #Neighbor Avoid Vars
        self.neighbor_avoid = neighbor_avoid    # Boolean to know we want to avoid our neighbors
        self._neighbor_tolerance = 0.5          # How close to neighbors do we get
        self._neighbor_tolerance_active = self._neighbor_tolerance  # active value used in movement. Adjusted depending on step of movement
        self._neighbor_collision_vector = None  # Vector to robot that will collide
        self._neighbor_delay = neighbor_delay   # seconds to wait before acting on neighbor in way
        self._neighbor_obstructed_time = None   # Time that a neighbor is in the way (used in manual move)
        self._neighbor_turning = 0              # Set to allow robot to turn before calculate collision, 1 = Left, 2 = right, 0 = not turning
        self._neighbor_turning_set = False      # Used to always turn the same way until path is clear
        self._neighbor_face_direction = None    # Which direction do I want to face
        self._neighbor_obstructed_manual = False # Boolean to allow to avoid neighbors while avoiding neighbors

        # status for the agent
        self._desired_location = None
        self._motion_complete = False
        self._destination_tolerance = destination_tolerance
        self._direction_facing = 0
        self._sync_move = sync_move     # Boolean used to activate sync move mode
        self._synce_state = 0           # State of this agent. 0 = not ready 1 = ready 2 = complete 4 = obstructed
        self._path_obstructed_time = None
        self._path_obstructed = False
        self._path_obstructed_laser = False
        self._path_obstructed_neighbor = False

    def pose_callback_(self, pose: PoseStamped):
        orientation = pose.pose.orientation
        x,y = pose.pose.position.x, pose.pose.position.y
        self.position = [x,y]
        self.direction_facing = self.get_angle_quad(orientation)

        if self.neighbor_avoid:
            self.path_obstructed_neighbor = self.is_neighbor_in_direction_(self.position, self.desired_location, self.neighbor_position)
        self.controller()

    def neighbor_pose_callback_(self, pose: PoseStamped, name):
        x,y = pose.pose.position.x, pose.pose.position.y
        self.neighbor_position[name] = [x,y]
    
    def lidar_callback_(self, msg: LaserScan):

        # Something is in the way if there is something between 2.8 and 3.469 radian
        if not self._laser_range_setup:
            self.setup_laser_config_(msg)

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
        left_side_range = np.array(msg.ranges[self._laser_forward_index:self._laser_left_index])

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
        # Yaw in gazebo will go from pi to -pi
        # Direction facing is in radians. pi is toward positive x (0 in gazebo)
        # 2pi is facing - x (pi in gazebo)
        # 0 is facing -x (-pi in gazebo)
        # Positive yaw (in gazebo) is counter clockwise, toward postive y+
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
    def desired_location(self):
        return self._desired_location
    @desired_location.setter
    def desired_location(self, location):
        # location = [x,y]
        self._desired_location = np.array(location)

    @property
    def path_obstructed_laser(self):
        return self._path_obstructed_laser
    @path_obstructed_laser.setter
    def path_obstructed_laser(self, value):
        self._path_obstructed_laser = bool(value)
        self.set_path_obstructed_()
    @property
    def path_obstructed_neighbor(self):
        return self._path_obstructed_neighbor
    @path_obstructed_neighbor.setter
    def path_obstructed_neighbor(self, value):
        self._path_obstructed_neighbor = bool(value)
        self.set_path_obstructed_()

    @property
    def path_obstructed(self):
        return self._path_obstructed
    
    def set_path_obstructed_(self):
        """
        Check the obsturction vars and if any are true, set _path_obstructed to True
        """
        values = [self.path_obstructed_laser, self.path_obstructed_neighbor]
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

        rf_index = self.laser_radian_index_(self.rf_radian, True)
        lf_index = self.laser_radian_index_(self.lf_radian, False)
        r_index = self.laser_radian_index_(self.r_radian, True)
        l_index = self.laser_radian_index_(self.l_radian, False)
        f_index = self.laser_radian_index_(self.f_radian, True)

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
            return int(np.floor((angle - self._laser_min_angle) / self._laser_angle_increment))
        else:
            return int(np.ceil((angle - self._laser_min_angle) / self._laser_angle_increment))

    def path_clear_laser_(self, desired_location, angle, magnitude, tolerance=2):
        """
        This function will check to see if the laser shows any more obstructions between us and our goal
        :param desired_location: an array of the position we want to get to
        :param angle: the angle between us and the goal
        :param magnitude: the magnitude between us and our goal
        :param tolerance: option parameter for the cutoff point for the clearance
        :return boolean: returns True if our path is clear and false if it is not
        """

        diff_angle = self.diff_angles(angle, self.direction_facing)
        idx = self.laser_radian_index_(diff_angle, True)
        return self._laser_scan[idx] > tolerance

    def scale_movement_(self, value):
        '''
        Creating cut off points for the movement
        '''
        new_value = max(-2.0, min(value, 2.0))
        if np.abs(new_value) > self._destination_tolerance and np.abs(new_value) < 0.5:
            new_value = 0.5 * (new_value / np.abs(new_value))
        elif np.abs(new_value) <= self._destination_tolerance:
            new_value = 0.0
        return new_value

    def move_direction(self, direction):
        '''
        Give the direction with magnitude you want to go.
        Will take current position and then pass the new position to move_to_position
        '''
        self.move_to_position(self.position + direction)

    def move_to_position(self, desired_location):
        """
        Start moving the robot to the desired direction. This handles when to drive vs when to turn. Also handles setting speeds for each
        :param desired_location: a position vector [x, y] you want to go to
        :return: None
        """

        self.desired_location = desired_location     # setting this value to be used else where
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
            elif x:
                if np.abs(z) > 0.001:
                    self.move_robot_(0.0, z)
                else:
                    self.move_robot_(x, z)
            else:
                if not self.motion_complete:
                    self.get_logger().info(f"{self.my_name} reached location")
                    self.motion_complete = True
                self.move_robot_(0.0, 0.0)

            if x and self.motion_complete:
                self.get_logger().info(f"{self.my_name} started moving again")
                self.motion_complete = False

        elif self.laser_avoid and self.path_obstructed_laser:
            self.move_around_laser_(desired_location)
        elif self.neighbor_avoid and self.path_obstructed_neighbor:
            self.move_around_neighbor_(desired_location)
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
            if self._laser_walk_around == 1 or self._laser_dynamic_right:
                self.walk_laser_right_()
                # Check to see if path is still blocked
                if not self._laser_obstructed_left and not self._laser_obstructed_forward:
                    self.get_logger().info("Laser Path Clear, checking destination")
                    self.path_obstructed_laser = not self.path_clear_laser_(desired_location, angle, magnitude)
            
            # go left
            elif self._laser_walk_around == 0 or self._laser_dynamic_left:
                self.walk_laser_left_()
                # Check to see if path is still blocked
                if not self._laser_obstructed_right and not self._laser_obstructed_forward:
                    self.get_logger().info("Laser Path Clear, checking destination")
                    self.path_obstructed_laser = not self.path_clear_laser_(desired_location, angle, magnitude)
            
            # dynamic system to decide left and right
            else:
                self.walk_laser_descide_()

        else:
            # waiting to see if obsturction moves
            self.move_robot_(0.0, 0.0)
        
        if not self.path_obstructed_laser and (self._laser_dynamic_left or self._laser_dynamic_right):
            self._laser_dynamic_right = False
            self._laser_dynamic_left = False

    def walk_laser_descide_(self):
        # right is negative and left is positive
        # 0 - middle = Right
        # middle - max = Left
        # Will find which side is the first one to be 10x the laser_distance and choose that side

        threshold = 1   # how many meters above distance is far enough
        middle = self._laser_forward_index
        obstruction = np.argmin(self._laser_scan)

        right_choice = 0
        for val in reversed(self._laser_scan[:obstruction]):
            if val >= threshold + self.laser_distance:
                break
            right_choice += 1

        left_choice = 0
        for val in self._laser_scan[obstruction+1:]:
            if val >= threshold + self.laser_distance:
                break
            left_choice += 1

        if right_choice < left_choice:
            self._laser_dynamic_right = True
            self.get_logger().info("Laser Decided to go right")
        else:
            self._laser_dynamic_left = True
            self.get_logger().info("Laser Decided to go left")

    def walk_laser_left_(self):
        # Rotate CCW until we can move again
        if self._laser_obstructed_forward:
            self.move_robot_(0.0, 1.0)
        # Drive Forward until clear
        elif self._laser_obstructed_right:
            self.move_robot_(1.0, 0.0)
        # turn back forward
        else:
            self.move_robot_(0.0, -1.0)

    def walk_laser_right_(self):
        # Rotate CW until we can move again
        if self._laser_obstructed_forward:
            self.move_robot_(0.0, -1.0)
        # Drive Forward until clear
        elif self._laser_obstructed_left:
            self.move_robot_(1.0, 0.0)
        # turn back forward
        else:
            self.move_robot_(0.0, 1.0)

    def is_neighbor_in_direction_(self, current_pos, desired_pos, neighbors):
        '''
        :param current_pos: is the coordinates of this robot
        :param desired_pos: is the coordinates of the desired location
        :param neighbors: is a dictionary { name: position, ect,}

        -Returns True if neighbor is in path
        '''
        if type(desired_pos) != type(None) and type(neighbors) != type(None):
            tolerance = self._neighbor_tolerance_active
            diameter = self._diameter

            current_pos = np.array(current_pos)
            desired_pos = np.array(desired_pos)
            
            # Calculate the direction vector from current to desired position
            direction_vector = desired_pos - current_pos
            direction_magnitude = np.linalg.norm(direction_vector)
            
            # If the direction magnitude is zero (i.e., current_pos == desired_pos), we can't move anywhere
            if direction_magnitude == 0:
                return False
            
            # Normalize the direction vector
            direction_vector_normalized = direction_vector / direction_magnitude
            
            # Check each neighbor
            for name, neighbor in neighbors.items():
                # Convert neighbor position to numpy array
                neighbor_pos = np.array(neighbor)
                
                # Calculate the vector from current position to the neighbor
                to_neighbor_vector = neighbor_pos - current_pos
                
                # Calculate the projection of the neighbor vector onto the direction vector
                projection_length = np.dot(to_neighbor_vector, direction_vector_normalized)
                
                # If the projection is less than zero, it means the neighbor is behind us
                # If the projection is larger than the magnitude of the direction, the neighbor is past the desired position
                if projection_length < 0 or projection_length > direction_magnitude:
                    continue 
                
                # Find the closest point on the path to the neighbor 
                closest_point_on_path = current_pos + projection_length * direction_vector_normalized
                
                # Calculate the perpendicular distance from the neighbor to the path (line)
                distance_to_path = np.linalg.norm(neighbor_pos - closest_point_on_path)
                
                # Check if the distance to the path is less than the diameter
                if distance_to_path <= diameter and np.linalg.norm(to_neighbor_vector) <= tolerance:
                    # check to make sure we are not being obstructed manually
                    if self._neighbor_obstructed_time == None:
                        self._neighbor_collision_vector = to_neighbor_vector
                    return True

        # If no neighbors were within tolerance in the right direction or no collision path detected, return False
        # pdb.set_trace()
        self._neighbor_tolerance_active = self._neighbor_tolerance
        self._neighbor_obstructed_manual = False
        self._neighbor_turning_set = False
        return False
        
    def is_neighbor_in_direction_manual_(self, current_pos, neighbors):
        """
        Parameters:
        :param current_pos: The current position of the robot as a tuple (x, y).
        :param neighbors: A list of tuples representing the positions of the neighbors.
        
        Returns:
        - True if a neighbor is close enough and a collision is possible, False otherwise.
        """
        tolerance = self._neighbor_tolerance
        diameter = self._diameter

        # Convert position to numpy array for easier vector operations
        current_pos = np.array(current_pos)
        
        # Calculate direction vector based on orientation (radians)
        direction_vector = self.direction_facing_vector_()
        
        # Check each neighbor
        for name, neighbor in neighbors.items():
            # Convert neighbor position to numpy array
            neighbor_pos = np.array(neighbor)
            
            # Calculate the vector from current position to the neighbor
            to_neighbor_vector = neighbor_pos - current_pos
            
            # Calculate the projection of the neighbor vector onto the direction vector
            projection_length = np.dot(to_neighbor_vector, direction_vector)
            
            # If the projection is less than zero, it means the neighbor is behind us
            # If the projection is larger than the magnitude of the direction, the neighbor is past the desired position
            if projection_length < 0:
                continue  # Skip this neighbor, as it doesn't lie on the forward path
            
            # Find the closest point on the path to the neighbor (the projection point)
            closest_point_on_path = current_pos + projection_length * direction_vector
            
            # Calculate the perpendicular distance from the neighbor to the path (line)
            distance_to_path = np.linalg.norm(neighbor_pos - closest_point_on_path)
            
            # Check if the distance to the path is less than the diameter
            if distance_to_path <= diameter and np.linalg.norm(to_neighbor_vector) <= tolerance:
                if self._neighbor_obstructed_time == None:
                    if not self._neighbor_obstructed_manual:
                        self._neighbor_obstructed_time = self._path_obstructed_time
                        self._neighbor_obstructed_manual = True
                        self._neighbor_tolerance_active = 2 * self._neighbor_tolerance
                    else:
                        self._neighbor_obstructed_time = datetime.datetime.now()
                        self._neighbor_collision_vector = to_neighbor_vector
                return True

        # If no neighbors were within tolerance in the right direction or no collision path detected, return False
        self._neighbor_obstructed_time = None
        return False

    def move_around_neighbor_(self, desired_location):
        if self._path_obstructed_time + datetime.timedelta(seconds=self._neighbor_delay) <= datetime.datetime.now():
            if self._neighbor_turning:
                # Shifting values to prevent wrap around
                new_direction_face = self.direction_facing
                new_face = self._neighbor_face_direction
                if self._neighbor_face_direction < np.pi/2 or self._neighbor_face_direction > 3 * np.pi /2:
                    new_face = (new_face + np.pi) % 2 * np.pi
                    new_direction_face = (new_direction_face + np.pi) % 2 * np.pi
                
                # going left
                if self._neighbor_turning == 1:
                    if np.abs(new_direction_face - new_face) < np.pi /3 and new_direction_face >= new_face:
                        self._neighbor_turning = False
                        self.move_robot_(0.0, 0.0)
                    else:
                        self.move_robot_(0.0, 1.0)
                # going right
                else:
                    if np.abs(new_direction_face - new_face) < np.pi /3 and new_direction_face <= new_face:
                        self._neighbor_turning = False
                        self.move_robot_(0.0, 0.0)
                    else:
                        self.move_robot_(0.0, -1.0)
                
            elif not self.is_neighbor_in_direction_manual_(self.position, self.neighbor_position):
                self.move_around_neighbor_movement_()
            else:
                if self._neighbor_obstructed_time + datetime.timedelta(seconds=self._neighbor_delay) <= datetime.datetime.now():
                    
                    if not self._neighbor_turning_set:
                        direction_heading = self.direction_facing_vector_()
                        # Get Cross product
                        cross_product = direction_heading[0] * self._neighbor_collision_vector[1] - direction_heading[1] * self._neighbor_collision_vector[0]
                        
                        # if > 0, we need to go right (CW)
                        if cross_product > 0:
                            print("Turn Right")
                            self._neighbor_turning_set = 2
                        # if < 0, we need to go left (CCW)
                        # if is is 0, we can do either. so lets go left
                        else:
                            print("Turn Left")
                            self._neighbor_turning_set = 1
                            
                    self._neighbor_turning = self._neighbor_turning_set
                    if self._neighbor_turning_set == 2:
                        self._neighbor_face_direction = (self.direction_facing - np.pi / 2) % (2 * np.pi)
                    else:
                        self._neighbor_face_direction = (self.direction_facing + np.pi / 2) % (2 * np.pi)
                else:
                    self.move_robot_(0.0, 0.0)
        else:
            self.move_robot_(0.0, 0.0)

    def move_around_neighbor_movement_(self):
        # thinking I may expand on this more later. Not very efficent, but it works
        self.move_robot_(1.0, 0.0)

    def direction_facing_vector_(self):
        return np.array([-np.cos(self.direction_facing), -np.sin(self.direction_facing)])

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

        # self.move_to_position(test)
        
        # self.move_robot_(0.0, 0.0)
        # return
        raise NotImplementedError('controller() not implemented for Agent base class.')

def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py yaml_load:=False robot_number:=2
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-t", "--test", default=[0,0], nargs='+', type=int, help="test var to pass in")
    script_args = parser.parse_args()

    rclpy.init(args=args)
    my_robot = Agent(int(script_args.index), np.array(script_args.neighbor), laser_avoid=False, neighbor_avoid=True)
    rclpy.spin(my_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()