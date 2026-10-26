#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Helper to check if a package exists
def package_available(package_name):
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False

def generate_launch_description():
    # Ensure we have a valid working directory to avoid Fast DDS XMLPARSER errors
    # Fast DDS tries to call getcwd() which fails if the CWD was deleted
    home_dir = os.path.expanduser('~')
    try:
        os.getcwd()
    except (FileNotFoundError, OSError):
        os.chdir(home_dir)

    # --- Per-robot ROS 2 namespace ------------------------------------------
    # Derive a fleet-safe namespace from the twin UUID so multiple UGV Beasts can
    # run on one ROS 2 graph without topic/node collisions:
    #     ugv_beast_<first 6 hex chars of the twin uuid>     e.g. ugv_beast_27dca7
    # The 'ugv_beast_' prefix guarantees a ROS-valid leading letter (a raw UUID
    # is invalid: it contains '-' and may start with a digit). Empty when
    # CYBERWAVE_TWIN_UUID is unset -> global topics (single-robot / dev).
    #
    # This MUST match mqtt_bridge.ros_topic_namespace.derive_robot_namespace so
    # the hardware nodes here and the mqtt_bridge land on identical topics. The
    # namespace is also passed to mqtt_bridge via the 'ros_namespace' param below.
    #
    # Override with CYBERWAVE_ROS_NAMESPACE (must be a valid ROS 2 name) to force
    # a specific namespace; otherwise it is derived from CYBERWAVE_TWIN_UUID.
    robot_namespace = os.getenv('CYBERWAVE_ROS_NAMESPACE', '').strip().strip('/')
    if not robot_namespace:
        _twin_uuid = os.getenv('CYBERWAVE_TWIN_UUID', '').replace('-', '').strip().lower()
        robot_namespace = f"ugv_beast_{_twin_uuid[:6]}" if _twin_uuid else ''

    def ns_child(child):
        """Nest a sub-namespace under the robot namespace (no-op if none).

        Used for nodes that already carry a sub-namespace (e.g. robot_state_publisher
        under 'ugv') so it becomes '<robot_namespace>/ugv'. Other hardware nodes use
        relative topic names and are placed directly under robot_namespace, so they
        need no per-topic prefixing here.
        """
        return f"{robot_namespace}/{child}" if robot_namespace else child

    # 1. Paths to packages and configurations
    ugv_bringup_dir = get_package_share_directory('ugv_bringup')
    ugv_vision_dir = get_package_share_directory('ugv_vision')
    ugv_description_dir = get_package_share_directory('ugv_description')
    mqtt_bridge_dir = get_package_share_directory('mqtt_bridge')
    ldlidar_dir = get_package_share_directory('ldlidar')
    
    # Check for optional packages
    has_joint_state_publisher = package_available('joint_state_publisher')
    has_usb_cam = package_available('usb_cam')
    has_image_proc = package_available('image_proc')
    has_ugv_base_node = package_available('ugv_base_node')
    
    # Configuration paths
    mqtt_config_path = os.path.join(mqtt_bridge_dir, 'config', 'params.yaml')
    
    # 2. Declare Arguments
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', 
        default_value='true',
        description='Whether to publish the tf from the original odom'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_ugv_beast_v1',
        description='Unique ID for the Cyberwave cloud'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to start the LiDAR driver'
    )
    
    camera_namespace_arg = DeclareLaunchArgument(
        name='camera_namespace', default_value=robot_namespace,
        description='Namespace for camera components (defaults to the per-robot '
                    'namespace so usb_cam publishes /<ns>/image_raw, matching the '
                    'namespaced subscription in mqtt_bridge)'
    )
    
    camera_container_arg = DeclareLaunchArgument(
        name='camera_container', default_value='',
        description='Existing container to load camera processing nodes into'
    )

    debug_logs_arg = DeclareLaunchArgument(
        'debug_logs',
        default_value='false',
        description='Enable debug logging for MQTT bridge (shows aiortc, aioice, etc. logs)'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true' if has_usb_cam else 'false',
        description='Whether to start the USB camera node (requires usb_cam package)'
    )

    # When true (default), mqtt_bridge owns the usb_cam process (CameraDeviceManager)
    # so pixel_format/resolution/fps can change at runtime and the respawn EBUSY race
    # is avoided. The launch-file usb_cam node below is then NOT started. Set false to
    # fall back to the static launch-file node.
    camera_managed_by_bridge_arg = DeclareLaunchArgument(
        'camera_managed_by_bridge',
        default_value='true',
        description='Let mqtt_bridge own the usb_cam lifecycle (no static usb_cam node)'
    )

    use_image_proc_arg = DeclareLaunchArgument(
        'use_image_proc',
        default_value='false',
        description=(
            'Start image_proc rectify_color_node for /image_rect '
            '(default false: teleop uses usb_cam /image_raw + mqtt_bridge only)'
        ),
    )
    
    use_joint_state_pub_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='true' if has_joint_state_publisher else 'false',
        description='Whether to start the joint_state_publisher (requires joint_state_publisher package)'
    )

    use_base_node_arg = DeclareLaunchArgument(
        'use_base_node',
        default_value='true' if has_ugv_base_node else 'false',
        description='Whether to start the base_node odometry calculator (requires ugv_base_node package)'
    )

    # 3. Core Hardware Node (Integrated Driver)
    # Handles Serial communication for both Telemetry and Commands
    # The node uses relative topic names (cmd_vel, ugv/led_ctrl, ...); placing it
    # under robot_namespace shifts ALL of them (including ones not remapped, e.g.
    # ugv/oled_ctrl) to /<ns>/... so they match the bridge's namespaced topics.
    # The old absolute remaps only forced topics back to global and are dropped;
    # with robot_namespace='' the relative names resolve to /cmd_vel etc. exactly
    # as before.
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_integrated_driver',
        name='ugv_bringup',
        namespace=robot_namespace or None,
        output='screen',
    )

    # 4. Lidar Driver
    # Includes the dedicated lidar launch file
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_dir, 'launch', 'ldlidar.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )

    # 5. Robot Description & Transforms
    # Publishes the 3D model and static transforms
    set_ugv_model = SetEnvironmentVariable('UGV_MODEL', 'ugv_beast')

    urdf_model_path = os.path.join(ugv_description_dir, 'urdf', 'ugv_beast.urdf')
    with open(urdf_model_path, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns_child('ugv'),
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=ns_child('ugv'),
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
        parameters=[{
            'robot_description': robot_description_content,
            'publish_default_positions': True,
        }]
    )

    # 6. Odometry Calculator
    # Computes raw odometry from wheel encoders
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        name='base_node',
        namespace=robot_namespace or None,
        condition=IfCondition(LaunchConfiguration('use_base_node')),
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
    )

    base_node_warning = LogInfo(
        msg='WARNING: ugv_base_node package not found - odometry will not be available. '
            'Build ugv_base_node with: colcon build --packages-select ugv_base_node',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_base_node'), "' == 'false'"]))
    )

    # 7. Cloud Connectivity (MQTT Bridge)
    # The bridge prefixes its ROS topics with 'ros_namespace' (via
    # resolve_ros_topic). Pass the same per-robot namespace used for the hardware
    # nodes so both sides resolve to identical /<ns>/... topics. The bridge node
    # itself is NOT placed under a launch namespace: it emits absolute /<ns>/...
    # names directly, so a launch namespace would have no effect (and risk
    # double-prefixing).
    mqtt_bridge_node = Node(
        package='mqtt_bridge',
        executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        parameters=[
            mqtt_config_path,
            {
                'robot_id': LaunchConfiguration('robot_id'),
                'debug_logs': LaunchConfiguration('debug_logs'),
                'ros_namespace': robot_namespace,
            }
        ],
        output='screen'
    )

    # 8. Video Streaming (Camera)
    # Load ugv_vision defaults and allow overrides from mqtt_bridge/config/params.yaml
    camera_param_file = os.path.join(ugv_vision_dir, 'config', 'params.yaml')
    camera_overrides = {}
    try:
        mqtt_params_file = os.path.join(mqtt_bridge_dir, 'config', 'params.yaml')
        with open(mqtt_params_file, 'r') as f:
            mqtt_params = yaml.safe_load(f) or {}
        camera_overrides = (
            mqtt_params.get('/mqtt_bridge_node', {})
            .get('ros__parameters', {})
            .get('camera', {})
        )
    except Exception:
        camera_overrides = {}

    # Static usb_cam node — only started when the bridge does NOT manage the
    # camera (camera_managed_by_bridge:=false). When the bridge owns it, the
    # CameraDeviceManager spawns usb_cam itself (no respawn race, runtime format
    # changes), so this node must stay out of the graph to avoid two owners of
    # /dev/video0.
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_camera'), "' == 'true' and '",
            LaunchConfiguration('camera_managed_by_bridge'), "' == 'false'"
        ])),
        parameters=[camera_param_file, camera_overrides],
        namespace=LaunchConfiguration('camera_namespace'),
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Image processing (rectify_color_node) — opt-in; off by default for cloud teleop
    image_processing_container = None
    load_composable_nodes = None
    
    if has_image_proc:
        camera_composable_nodes = [
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                namespace=LaunchConfiguration('camera_namespace'),
                remappings=[
                    ('image', 'image_raw'),
                    ('image_rect', 'image_rect')
                ],
            )
        ]

        image_processing_container = ComposableNodeContainer(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_image_proc'), "' == 'true' and '",
                LaunchConfiguration('camera_container'), "' == '' and '",
                LaunchConfiguration('use_camera'), "' == 'true'"
            ])),
            name='image_proc_container',
            namespace=LaunchConfiguration('camera_namespace'),
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=camera_composable_nodes,
            output='screen'
        )

        load_composable_nodes = LoadComposableNodes(
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_image_proc'), "' == 'true' and '",
                LaunchConfiguration('camera_container'), "' != '' and '",
                LaunchConfiguration('use_camera'), "' == 'true'"
            ])),
            composable_node_descriptions=camera_composable_nodes,
            target_container=LaunchConfiguration('camera_container'),
        )

    # Build the launch description with all nodes
    ld = LaunchDescription([
        set_ugv_model,
        pub_odom_tf_arg,
        robot_id_arg,
        use_lidar_arg,
        camera_namespace_arg,
        camera_container_arg,
        debug_logs_arg,
        use_camera_arg,
        camera_managed_by_bridge_arg,
        use_image_proc_arg,
        use_joint_state_pub_arg,
        use_base_node_arg,
        bringup_node,
        laser_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        base_node_warning,
        base_node,
        mqtt_bridge_node,
        camera_node,
    ])
    
    # Add image processing nodes only if available
    if image_processing_container is not None:
        ld.add_action(image_processing_container)
    if load_composable_nodes is not None:
        ld.add_action(load_composable_nodes)
    
    return ld
