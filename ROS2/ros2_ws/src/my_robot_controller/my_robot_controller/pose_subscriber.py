#!/usr/bin/env python3
import rclpy
from rclpy.node  import Node
from turtlesim.msg import Pose

class PoseSubcriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", 
                                                        self.poseCallback, 10)
    

    def poseCallback(self, msg: Pose):
        self.get_logger().info(str(msg))



def main(args=None):
    rclpy.init(args=args)
    node = PoseSubcriberNode()
    rclpy.spin(node)
    rclpy.shutdown()