
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty.world')

    # Headless Gazebo server only (no GUI)
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot in Gazebo
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'description', 'turtlebot3_burger.urdf')
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Robot state publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
    )

    # RViz with config
    rviz_config = os.path.join(os.path.dirname(__file__), '..', 'config', 'gazebo.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Robot X position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Robot Y position'),
        start_gazebo,
        spawn_entity,
        robot_state_publisher,
        rviz
    ])
