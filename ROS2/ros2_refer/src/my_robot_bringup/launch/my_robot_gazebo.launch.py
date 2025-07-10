from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    ])

    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_bringup'),
        'config',
        'gazebo_bridge.yaml'
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'rviz',
        'urdf_config.rviz'
    ])

    # Process URDF using xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path
    ])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # Spawn robot into Gazebo
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # ROS-Gazebo parameter bridge
    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[gazebo_config_path],
    output='screen',
    name='gz_parameter_bridge'
)

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot,
        bridge_node,
        rviz_node
    ])