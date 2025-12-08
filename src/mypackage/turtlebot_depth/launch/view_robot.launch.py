import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = LaunchConfiguration('urdf_file', default=os.path.join(
        os.path.dirname(__file__), '..', 'description', 'turtlebot3_burger.urdf'))

    # Path to RViz config
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'turtlebot3.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                os.path.dirname(__file__), '..', 'description', 'turtlebot3_burger.urdf'),
            description='Path to URDF file'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])
