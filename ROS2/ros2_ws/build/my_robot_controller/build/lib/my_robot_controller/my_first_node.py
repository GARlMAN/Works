#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


#creating a node using a class by inherting a node
class myNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hellow world from ROS2")
    
    

def main(args=None):
    rclpy.init(args=args) #initializxing the ros2 communications
    
    #create the node
    node = myNode()
    rclpy.spin(node) #this node is kept alive indefineity until you kill
    rclpy.shutdown()   #ends the communciations and distroy node


if __name__ == "__main__":
    main()