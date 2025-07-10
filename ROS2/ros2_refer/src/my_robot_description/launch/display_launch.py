from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'rviz',
        'urdf_config.rviz'
    ])

    return LaunchDescription([
        # Launch Ignition Gazebo Fortress (now Gazebo Sim)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', '4', 'empty.sdf'],
            output='screen'
        ),

        # Publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Spawn robot using ros_gz
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'my_robot',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),

    #    Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )
    ])
