#!/usr/bin/env python3
"""Launch file for MQTT Bridge node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for MQTT bridge."""
    
    # Clear screen for a clean CLI experience
    os.system('clear')
    
    # Get package directory
    pkg_dir = get_package_share_directory('mqtt_bridge')
    
    # Clean CLI Banner
    print("\n\033[1;94m" + "="*60)
    print("   \033[1;96m🌐  CYBERWAVE MQTT BRIDGE - INITIALIZING SESSION\033[1;94m")
    print("="*60 + "\033[0m")
    
    # List available mappings for "Clean CLI" experience
    mappings_dir = os.path.join(pkg_dir, 'config', 'mappings')
    if os.path.exists(mappings_dir):
        available = [f.replace('.yaml', '') for f in os.listdir(mappings_dir) if f.endswith('.yaml')]
        print(f"   \033[1;32m● Available Mappings:\033[0m {', '.join(available)}")
    
    import sys
    passed_robot_id = next((arg.split(':=')[1] for arg in sys.argv if arg.startswith('robot_id:=')), None)
    if passed_robot_id:
        print(f"   \033[1;33m➜  Selected Robot ID:\033[0m {passed_robot_id}")
    else:
        print(f"   \033[1;33m➜  Selected Robot ID:\033[0m robot_ugv_beast_v1 (default)")
    
    print(f"   \033[1;32m● Launch Command:\033[0m ros2 launch mqtt_bridge mqtt_bridge.launch.py robot_id:=<id>")
    print("\033[1;94m" + "="*60 + "\033[0m\n")

    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_ugv_beast_v1',
        description='Robot mapping ID (Options: robot_ugv_beast_v1, or default).'
    )
    
    # allow overriding the prefix used to execute the node (useful to force a
    # specific interpreter or add extra shell-level instrumentation). This is a
    # plain string substitution inserted in Node(prefix=[...]).
    python_prefix = LaunchConfiguration('python_prefix', default='')
    robot_id = LaunchConfiguration('robot_id')

    # Path to params file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # MQTT Bridge Node
    mqtt_bridge_node = Node(
        package='mqtt_bridge',
        executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        output='screen',
        prefix=[python_prefix],
        parameters=[
            params_file,
            {'robot_id': robot_id}  # Override robot_id from launch argument
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        robot_id_arg,
        LogInfo(msg=["\033[92m[INFO] Initializing MQTT Bridge for Robot ID: \033[0m", robot_id]),
        LogInfo(msg=["\033[92m[INFO] Loading parameters from: \033[0m", params_file]),
        mqtt_bridge_node,
    ])

