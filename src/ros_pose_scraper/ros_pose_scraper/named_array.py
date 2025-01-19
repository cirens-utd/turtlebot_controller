#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped

class Named_Array(Node):
    def __init__(self, *args, robot_name="robot", source_topic="/encoded_poses", new_name="turtlebot"):
        super().__init__("Named_Array")
        self.robot_name = robot_name
        self.new_name = new_name

        self.get_logger().info("Named Array Parser Node Started")

        # Create an array of publishers we need to push to.
        self.pose_publisher_ = {}

        # Create subscriber
        self.pose_sub_ = self.create_subscription(
            PoseArray,
            f"{source_topic}",
            self.pose_sub_callback, 10
        )

    def pose_sub_callback(self, msg: PoseArray):
        # This message will parse the encoded PoseArray and publish each robot on its own PoseStamped topic
        # the robot name should be in the formate {robot_name}X (robot1, robot2, ect)
        # the frameID should have the order of names of each pose deleminated by ','
        # : geometry_msgs.msg.PoseArray msg: Takes PoseArray msg as argument

        names = msg.header.frame_id.split(',')
        for index,name in enumerate(names):
            # make sure publisher is created
            if name[:len(self.robot_name)] == self.robot_name:
                if name not in self.pose_publisher_:
                    number = name[len(self.robot_name):]
                    self.pose_publisher_[name] = self.create_publisher(
                        PoseStamped, 
                        f"/vrpn_mocap/{self.new_name + number}/pose",
                        10
                        )
                    self.get_logger().info(f"Created publisher for {name}")
                # publish new topic
                pose = PoseStamped()
                pose.header.frame_id = name
                pose.header.stamp = msg.header.stamp
                pose.pose.position = msg.poses[index].position
                pose.pose.orientation = msg.poses[index].orientation
                self.pose_publisher_[name].publish(pose)
    
def main(args=None):
    rclpy.init(args=args)
    named_array_parser = Named_Array()
    rclpy.spin(named_array_parser)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
