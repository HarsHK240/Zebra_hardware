import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Locate the package
    pkg_share = FindPackageShare('zebra')
    
    # 2. Path to XACRO and Controller Config
    urdf_xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'zebra.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'zebra_controllers.yaml'])

    # 3. Build robot_description (xacro)
    # Note: 'use_sim_time' is removed or set to false. 
    # If your xacro has a conditional argument for hardware (like 'use_ros2_control'), add it here.
    robot_description_content = Command([
        'xacro ', urdf_xacro_file,
        ' use_sim_time:=false' 
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # 4. Robot State Publisher
    # Publishes TF transforms based on the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # 5. ROS2 Control Node (The Heart of Hardware Interface)
    # This node loads your C++ hardware interface and manages the controllers.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
        remappings=[
            # Remap the controller's expected input to the standard /cmd_vel
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
        ]
    )

    # 6. Spawners
    # These tell the controller manager (running inside control_node) to start specific controllers.
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
    ])