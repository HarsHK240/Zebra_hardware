import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_bot_description = get_package_share_directory('zebra')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Path to the Nav2 parameters file for mapless navigation
    nav2_params_file = os.path.join(pkg_my_bot_description, 'config', 'nav2_param.yaml')
    
    # Path to the EKF config file for sensor fusion
    ekf_config_file = os.path.join(pkg_my_bot_description, 'config', 'ekf.yaml')

    # --- Actions ---

    # 1. Include your robot's main launch file (starts Gazebo, ros2_control, etc.)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_bot_description, 'launch', 'display.launch.py')
        )
    )

    # 2. LOCAL EKF: Fuses Wheel Odom + IMU (Provides odom -> base_link tf)
    # The 'name' must match the YAML block 'ekf_filter_node_odom'
    ekf_local_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/local')]
    )

    # 3. GLOBAL EKF: Fuses Wheel Odom + IMU + GPS (Provides map -> odom tf)
    # The 'name' must match the YAML block 'ekf_filter_node_map'
    ekf_global_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/global')]
    )

    # 4. NAVSAT TRANSFORM: Converts GPS Lat/Lon to Odom X/Y
    start_navsat_transform_cmd = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}],
        remappings=[
            ('/imu', '/imu'),          # Map standard /imu to /imu/data
            ('/gps/fix', '/gps'),       # Ensure this matches your Gazebo topic
            ('/odometry/filtered', '/odometry/global'), # Feeds global position back to this node
            ('/odometry/gps', '/odometry/gps')
        ]
    )

    # 5. Include the Nav2 bringup launch file in mapless mode
    # Uses navigation_launch.py to avoid loading map_server/amcl
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': 'true',
            # We do NOT pass 'map' here because navigation_launch doesn't use it
        }.items()
    )
    
    odom_launch = Node(
        package='my_bot_description',
        executable='odom.py',
        name='odom_script',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    return LaunchDescription([
        robot_launch,
        ekf_local_cmd,   # Local EKF
        ekf_global_cmd,  # Global EKF
        start_navsat_transform_cmd,
        nav2_launch,
        odom_launch,
    ])
