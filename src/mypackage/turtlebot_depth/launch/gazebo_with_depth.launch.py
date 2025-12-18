#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    log_level = LaunchConfiguration('log_level', default='info')

    # Get package directories
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    turtlebot_depth_pkg = get_package_share_directory('turtlebot_depth')
    depth_anything_pkg = get_package_share_directory('depth_anything_v2_ros2')
    rtabmap_launch_pkg = get_package_share_directory('rtabmap_launch')

    # Get paths to patched files
    patched_files_dir = os.path.join(turtlebot_depth_pkg, 'depth_anything_patches')

    # Paths
    world_file = os.path.join(tb3_gazebo_pkg, 'worlds', 'turtlebot3_house.world')
    urdf_path = os.path.join(turtlebot_depth_pkg, 'description', 'turtlebot3_burger.urdf')
    rviz_config = os.path.join(turtlebot_depth_pkg, 'config', 'gazebo.rviz')
    wait_script = os.path.join(turtlebot_depth_pkg, 'scripts', 'wait_for_topics.sh')

    # Depth Anything configuration
    depth_params_file = os.path.join(turtlebot_depth_pkg, 'config', 'depth_anything_params.yaml')
    default_model_file = os.path.join(turtlebot_depth_pkg, 'models', 'depth_anything_v2_metric_hypersim_vits.pth')
    model_file = LaunchConfiguration('model_file')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot X position'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot Y position'
    )

    declare_model_file = DeclareLaunchArgument(
        'model_file',
        default_value=default_model_file,
        description='Full path to the Depth Anything V2 model file'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (info, debug, warn, error)'
    )

    # Headless Gazebo server only (no GUI)
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn robot in Gazebo
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
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # Configure Depth Anything parameters
    param_substitutions = {
        'model_file': model_file,
    }

    configured_params = RewrittenYaml(
        source_file=depth_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Depth Anything V2 node
    depth_anything_node = Node(
        package='depth_anything_v2_ros2',
        executable='depth_anything_v2_ros2',
        name='depth_anything',
        parameters=[
            configured_params,
            {'use_sim_time': use_sim_time}
        ],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['depth_anything:=', log_level]
        ]
    )

    # Wait for sensor topics before starting RTAB-Map
    wait_for_topics = ExecuteProcess(
        cmd=['bash', wait_script],
        output='screen',
        name='wait_for_topics'
    )

    # RTAB-Map SLAM with message synchronization
    # This will be launched AFTER wait_for_topics completes
    start_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_launch_pkg, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            # Simulation time
            'use_sim_time': 'true',

            # Input topics
            'rgb_topic': '/sim_camera/image_raw',
            'depth_topic': '/sim_camera/depth',
            'camera_info_topic': '/sim_camera/camera_info',
            'odom_topic': '/odom',
            'imu_topic': '/imu/data',
            # 'scan_topic': '/scan',  # LiDAR disabled - using pure RGB-D SLAM

            # Frame IDs
            'frame_id': 'base_footprint',  # Robot base frame (not camera!)
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # Synchronization settings
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.1',  # 100ms window
            'queue_size': '10',

            # QoS settings (2 = Best Effort to match Gazebo camera)
            'qos': '2',

            # Subscriptions
            'subscribe_depth': 'true',
            'subscribe_rgb': 'true',
            'subscribe_scan': 'false',  # LiDAR disabled - pure RGB-D SLAM
            'rgbd_sync': 'true',  # Enable RGB-D synchronization

            # IMU settings
            'wait_imu_to_init': 'true',

            # SLAM parameters
            'Mem/IncrementalMemory': 'true',  # Mapping mode
            'Rtabmap/DetectionRate': '1.0',  # Process every frame

            # Disable visual odometry (we have wheel odometry)
            'visual_odometry': 'false',

            # Enable rtabmap_viz (RTAB-Map's built-in visualization)
            'rtabmap_viz': 'true',

            # Logging
            'log_level': 'info',
        }.items()
    )

    # Event handler to start RTAB-Map only after topics are available
    delayed_rtabmap = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_topics,
            on_exit=[start_rtabmap]
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_model_file,
        declare_log_level,
        start_gazebo,
        spawn_entity,
        robot_state_publisher,
        # rviz,  # Removed - using rtabmap_viz instead
        depth_anything_node,
        wait_for_topics,      # Wait for all sensor topics
        delayed_rtabmap       # Launch RTAB-Map after topics are ready
    ])
