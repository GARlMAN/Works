#!/usr/bin/env python3
import rclpy
from rclpy.node  import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class turtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle controller has been started")

        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", 
                                                        self.poseCallback, 10) #need to ask what the 10 does
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    

    def poseCallback(self, pose: Pose):
        cmds = Twist()

        # Move forward at constant speed
        cmds.linear.x = 2.0
        self.cmd_vel_pub.publish(cmds)
        # Angular velocity increases with distance from center
        # This causes a spiral-like turning
        center_x, center_y = 5.5, 5.5
        dist = math.sqrt((pose.x - center_x)**2 + (pose.y - center_y)**2)

        cmds.angular.z = 0.5 + 0.2 * dist

        self.cmd_vel_pub.publish(cmds)



def main(args=None):
    rclpy.init(args=args)
    node = turtleController()
    rclpy.spin(node)
    rclpy.shutdown()