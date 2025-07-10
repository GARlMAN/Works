#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file - equivalent to $(find-pkg-share ...)
    config_path = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'catch_them_all_config.yaml'
    )
    
    return LaunchDescription([
        # XML: <node pkg="turtlesim" exec="turtlesim_node" name="turtlesim" />
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # XML: <node pkg="catch_turtle_sim" exec="turtle_controller" name="turtle_controller" />
        Node(
            package='catch_turtle_sim',
            executable='turtle_controller',
            name='turtle_controller'
        ),
        
        # XML: <node pkg="catch_turtle_sim" exec="spawner" name="spawner" output="screen">
        #          <param from="$(find-pkg-share my_robot_bringup)/config/catch_them_all_config.yaml" />
        #      </node>
        Node(
            package='catch_turtle_sim',
            executable='spawner',
            name='spawner',
            output='screen',
            parameters=[config_path]
        )
    ])