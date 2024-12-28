import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Dynamically determine the path to the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'my_robot.urdf.xacro'
    )

    # Process the Xacro file
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True  # Simulated time for Gazebo
        }]
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Spawn the robot entity in Gazebo using the robot description from the topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])

