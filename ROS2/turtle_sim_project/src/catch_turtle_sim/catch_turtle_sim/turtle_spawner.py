#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from functools import partial
import random
import math
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        self.declare_parameter("turtle_name_prefix", "turtle")
        self.declare_parameter("spawn_frequency", 1.0)
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value
        self.spawn_frequency = self.get_parameter("spawn_frequency").value




        # Publisher to broadcast list of currently alive turtles
        self.alive_turtles_publisher = self.create_publisher(
            TurtleArray, "/alive_turtles", 10
        )
        self.alive_turtles = []  # Local list to store alive turtles' info

        # Client to call the /spawn service of turtlesim
        self.spawn_client = self.create_client(Spawn, "/spawn")


        self.counter = 0  # Counter to assign unique turtle names

        # Timer to periodically spawn a new turtle every 2 seconds
        self.spawn_turtle_timer = self.create_timer(1.0/self.spawn_frequency, self.spawn_new_turtle)

        # Service server to respond to turtle catch requests
        self.catch_turtle_service = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

        # Client to call the /kill service to remove a turtle
        self.kill_client = self.create_client(Kill, "/kill")

    # Called when a turtle is requested to be "caught" (removed)
    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        self.call_kill_service(request.name)
        response.success = True  # Always returns success for simplicity
        return response

    # Sends asynchronous kill request to /kill service
    def call_kill_service(self, turtle_name):
        # Wait until /kill service is available
        while not self.kill_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill service to start")

        # Prepare the request with the turtle's name
        request = Kill.Request()
        request.name = turtle_name

        # Send the request and register callback
        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill_service, request=request))

    # Called after the /kill service completes
    def callback_call_kill_service(self, future, request: Kill.Request):
        try:
            _ = future.result()  # Ensure no exception occurred
            # Remove the turtle from local list by matching name
            self.alive_turtles = [
                turtle for turtle in self.alive_turtles if turtle.name != request.name
            ]
            self.publish_alive_turtles()  # Publish updated turtle list
        except Exception as e:
            self.get_logger().error(f"Kill service failed: {e}")

    # Publishes the current list of alive turtles to /alive_turtles
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtles_publisher.publish(msg)

    # Spawns a new turtle at random coordinates
    def spawn_new_turtle(self):
        self.counter += 1
        name = self.turtle_name_prefix + str(self.counter)

        # Random position and orientation
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, math.pi)

        # Call the spawn service with generated values
        self.call_spawn_service(name, x, y, theta)

    # Sends a request to /spawn to create a new turtle
    def call_spawn_service(self, turtle_name, x, y, theta):
        while not self.spawn_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for spawn service to start")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn_service, request=request))

    # Called when a turtle is successfully spawned
    def callback_call_spawn_service(self, future, request: Spawn.Request):
        try:
            response: Spawn.Response = future.result()
            if response.name != "":
                self.get_logger().info("New alive turtle: " + response.name)

                # Create and store turtle info
                new_turtle = Turtle()
                new_turtle.name = request.name
                new_turtle.x = request.x
                new_turtle.y = request.y
                new_turtle.theta = request.theta

                self.alive_turtles.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")

# Main entry point
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
