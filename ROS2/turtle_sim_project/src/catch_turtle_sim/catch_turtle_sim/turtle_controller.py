#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle controller has been started")

        self.turtle_to_catch: Turtle = None  # Currently targeted turtle
        self.pose: Pose = None  # Current pose of turtle1

        # Publishes velocity commands to turtle1
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Subscribes to turtle1's pose updates
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10
        )

        # Subscribes to list of currently alive turtles
        self.alive_turtle_subscriber = self.create_subscription(
            TurtleArray, "/alive_turtles", self.call_back_alive_turtle_subscriber, 10
        )

        # Timer for control loop (100 Hz)
        self.control_loop_timer = self.create_timer(0.01, self.callback_loop)

        # Client for the CatchTurtle service
        self.catch_turtle_client = self.create_client(CatchTurtle, "catch_turtle")

    # Calls the catch_turtle service to "kill" a turtle
    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for catch_turtle service...")

        # Prepare request
        request = CatchTurtle.Request()
        request.name = turtle_name

        # Asynchronously call the service
        future = self.catch_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, request=request))

    # Callback after catch_turtle service completes
    def callback_call_catch_turtle_service(self, future, request: CatchTurtle.Request):
        try:
            response: CatchTurtle.Response = future.result()
            if not response.success:
                self.get_logger().error(f"Turtle {request.name} could not be removed")
        except Exception as e:
            self.get_logger().error(f"Catch turtle service failed: {e}")

    # Called when a new list of alive turtles is received
    def call_back_alive_turtle_subscriber(self, msg: TurtleArray):
        # Only pick a new target if we don't already have one (LOCK-ON)
        if self.turtle_to_catch is not None:
            return  # Keep current target, don't switch
            
        # Set target turtle to the closest one (only when we don't have a target)
        if len(msg.turtles) > 0:
            # Find the turtle with the minimum distance from turtle1
            closest_turtle = None
            min_distance = float('inf')

            for turtle in msg.turtles:
                dx = turtle.x - self.pose.x
                dy = turtle.y - self.pose.y
                dist = math.sqrt(dx**2 + dy**2)

                if dist < min_distance:
                    min_distance = dist
                    closest_turtle = turtle

            self.turtle_to_catch = closest_turtle
        else:
            self.turtle_to_catch = None

    # Called whenever turtle1's pose is updated
    def callback_pose(self, msg: Pose):
        self.pose = msg

    # Main control loop — moves turtle1 toward the target
    def callback_loop(self):
        if self.pose is None or self.turtle_to_catch is None:
            return  # Do nothing if state is not ready

        # Calculate distance to target
        dx = self.turtle_to_catch.x - self.pose.x
        dy = self.turtle_to_catch.y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)

        cmd = Twist()  # Velocity command to be published

        if distance > 0.5:
            # Move forward proportional to distance
            cmd.linear.x = 4.0 * distance

            # Calculate angle to face target
            goal_theta = math.atan2(dy, dx)
            diff = goal_theta - self.pose.theta

            # Normalize angle difference to [-pi, pi]
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            # Rotate proportional to angular difference
            cmd.angular.z = 12.0 * diff
        else:
            # Close enough — stop and call catch service
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch.name)
            self.turtle_to_catch = None  # Reset target

        self.cmd_vel_publisher.publish(cmd)

# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
