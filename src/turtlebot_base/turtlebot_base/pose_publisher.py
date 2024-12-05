import rclpy
from rclpy.node import Node
from rclpy.node import Node
from ignition_msgs.msg import Pose_V 
from geometry_msgs.msg import PoseArray

class Pose_Publisher(Node):
    def __init__(self, world, *args): 
        super().__init__("pose_publisher")

        # Subsriber
        self.subscrition = self.create_subscription(
            Pose_V, 
            f'/world/{world}/dynamic_pose/info',
            self.listener_callback,
            10)
        self.publisher = {}
        '''
        self.publisher['test'] =  = self.create_publisher(
            PoseArray,
            f'/vrpn_mocap/turtlebot{i+1}/pose',
            10
        )
        '''


    def listener_callback(self, pose: Pose_V):
        print(pose)
        
def main(args=None):
    rclpy.init(args=args)
    ros_publisher = Pose_Publisher("empty_world")
    rclpy.spin(ros_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()