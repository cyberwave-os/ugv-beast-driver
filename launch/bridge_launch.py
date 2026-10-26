"""ROS2 launch description for the mqtt_bridge node.

This file launches the single `mqtt_bridge_node` ROS2 node and loads 
`config/params.yaml` as the node parameters. It intentionally keeps
the launch simple and documents a few common override points:

- `python_prefix` (launch argument): optional prefix used when executing the
  node process. This is handy when you want to force a specific Python
  interpreter (for example a virtualenv) by passing something like
  "python_prefix:=/path/to/venv/bin/python -u" on the ros2 launch command
  line. When empty the node is launched normally by the ROS2 launcher.

- Launch with a virtualenv/python prefix (keeps output unbuffered):
  $ ros2 launch mqtt_bridge bridge_launch.py python_prefix:="/path/to/venv/bin/python -u"

The node reads mapping and bridge configuration from the package's
`config/params.yaml` by default; you can override parameters or the
configuration file via standard ROS2 mechanisms if needed.
"""

import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    # locate the default params.yaml shipped with the package
    pkg_share = get_package_share_directory('mqtt_bridge')
    cfg = os.path.join(pkg_share, 'config', 'params.yaml')

    # allow overriding the prefix used to execute the node (useful to force a
    # specific interpreter or add extra shell-level instrumentation). This is a
    # plain string substitution inserted in Node(prefix=[...]).
    python_prefix = LaunchConfiguration('python_prefix', default='')
    
    # Allow overriding log level
    log_level = LaunchConfiguration('log_level', default='info')

    # Create the node action. The `parameters` argument points to the YAML
    # file above; ROS2 will load parameters from it into the node at startup.
    mqtt_bridge_node = Node(
        package='mqtt_bridge',
        executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        output='screen',
        prefix=[python_prefix],
        parameters=[cfg],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Print a helpful message so users launching from a terminal see which
    # configuration file was loaded.
    print("MQTT Bridge node launched with configuration from:", cfg)

    # Declare the launch argument so it can be set on the `ros2 launch`
    # command line (see usage examples above).
    declare_prefix = DeclareLaunchArgument(
        'python_prefix',
        default_value='',
        description='Prefix used to execute the node (set to a venv python to force interpreter)'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the node (use debug for troubleshooting upstream/downstream)'
    )

    return LaunchDescription([declare_prefix, declare_log_level, mqtt_bridge_node])
