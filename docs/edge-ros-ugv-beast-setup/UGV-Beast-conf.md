UGV-Beast-conf.md#UGV Beast ROS 2 DocumentationThis comprehensive guide covers the technical architecture, topic mapping, and operational commands for the **UGV Beast** robot using ROS 2.

---

##📋 Table of Contents1. [System Architecture](https://www.google.com/search?q=%231-system-architecture) 2. [Hardware Control Logic](https://www.google.com/search?q=%232-hardware-control-logic) 3. [Topic Map & Interfaces](https://www.google.com/search?q=%233-topic-map--interfaces) 4. [Joint Configuration (URDF)](https://www.google.com/search?q=%234-joint-configuration-urdf) 5. [Minimum Setup Control Guide](https://www.google.com/search?q=%235-minimum-setup-control-guide) 6. [Advanced Control: Joint Trajectories](https://www.google.com/search?q=%236-advanced-control-joint-trajectories) 7. [Technical Best Practices](https://www.google.com/search?q=%237-technical-best-practices)

---

##1. System ArchitectureThe UGV Beast utilizes a standard ROS 2 hardware abstraction layer to bridge high-level software with physical actuators.

1. **The Command Layer:** High-level messages are published to topics like `/cmd_vel`.
2. **The Bridge (`ugv_driver`):** This node acts as the primary translator. It subscribes to ROS topics and converts high-level math (e.g., 0.5 \text{ m/s}) into low-level serial bytes.
3. **The Communication Link:** Data is sent via **Serial (TTL/UART)** from the Raspberry Pi to the onboard ESP32.
4. **The Execution Layer (ESP32):** The ESP32 (Slave) calculates PID loops for motor speed and manages GPIO/UART for LEDs and Pan-Tilt servos.

---

##2. Hardware Control LogicKey launch commands to initialize the hardware:

- **`ros2 run ugv_bringup ugv_driver`**: The most critical command. It opens the serial port (typically `/dev/ttyS0` or USB) and allows the Raspberry Pi to "see" and control the hardware.
- **`ros2 launch ugv_tools teleop_twist_joy.launch.py`**: Launches the Joystick (HID) to `Twist` conversion. Includes a **Deadman's switch** (The 'R' button lock/unlock) for safety.

---

##3. Topic Map & InterfacesBelow are the topics exposed by the `ugv_driver`.

| Topic Name            | Direction | Message Type                 | Description                                           |
| --------------------- | --------- | ---------------------------- | ----------------------------------------------------- |
| `/cmd_vel`            | **IN**    | `geometry_msgs/Twist`        | **Main Control:** Linear x (speed), Angular z (turn). |
| `/ugv/led_ctrl`       | **IN**    | `std_msgs/Float32MultiArray` | `[chassis, camera]` LEDs (0–255 brightness).          |
| `/ugv/pt_ctrl`        | **IN**    | `std_msgs/Float32MultiArray` | **Pan-Tilt:** Raw angles (Yaw/Pitch) in degrees.      |
| `/ugv/oled_ctrl`      | **IN**    | `std_msgs/String`            | Sends text to the 0.91" onboard OLED.                 |
| `/ugv/joint_states`   | **OUT**   | `sensor_msgs/JointState`     | **Feedback:** Position/velocity of wheels and servos. |
| `/ugv/imu`            | **OUT**   | `sensor_msgs/Imu`            | **Balance:** Raw 9-axis IMU data.                     |
| `/ugv/battery_status` | **OUT**   | `sensor_msgs/BatteryState`   | **Power:** Reports voltage levels.                    |
| `/ugv/odom`           | **OUT**   | `nav_msgs/Odometry`          | **Position:** Calculated distance via motor encoders. |

---

##4. Joint Configuration (URDF)The `ugv_description` defines specific joints for 3D spatial awareness in RViz.

###Chassis Joints (Actuated via Encoders)Simulated as four wheels to calculate odometry for the tracked system:

- `left_up_wheel_link_joint` / `left_down_wheel_link_joint`
- `right_up_wheel_link_joint` / `right_down_wheel_link_joint`

###Pan-Tilt Joints (Actuated via Servos)\* **`pt_base_link_to_pt_link1` (Pan/Yaw):** Horizontal camera rotation.

- **`pt_link1_to_pt_link2` (Tilt/Pitch):** Vertical camera rotation.

###Fixed Joints (Structural)Required for the Transform Tree (TF):

- `base_footprint_joint`: Chassis to floor connection.
- `lidar_joint` / `camera_joint`: Sensor-to-body alignment.

---

###Step B: Command Palette (New Terminal)\* **Move Forward:**
`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

- **Stop:**
  `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`
- **Pivot Right:**
  `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: -0.5, y: 0.0, z: 0.0}}"`
- **Lights ON (Full):**
  `ros2 topic pub --once /ugv/led_ctrl std_msgs/msg/Float32MultiArray "{data: [255.0, 255.0]}"`
- **Center Pan-Tilt:**
  `ros2 topic pub --once /ugv/pt_ctrl std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0]}"`

---

##6. Advanced Control: Joint TrajectoriesFor high-precision mapping or simulation, you can target joints directly using the `scaled_joint_trajectory_controller`.

###Command all wheels forward:```bash
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
joint_names: ['left_up_wheel_link_joint', 'left_down_wheel_link_joint', 'right_up_wheel_link_joint', 'right_down_wheel_link_joint'],
points: [{
positions: [0.0, 0.0, 0.0, 0.0],
velocities: [1.0, 1.0, 1.0, 1.0],
time_from_start: {sec: 1, nanosec: 0}
}]
}"

````




## EDGE Configuration

### SSH connection

accordingly to this [UGV Beast PI ROS2](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2)

enter ninto the raspberry to set up the wifi and ssh connection. follow the previous link for a better explaination
```bash
cd ~/ugv_rpi
cd AccessPopup/
sudo chmod +x installconfig.sh
````

2.  ssh

```bash
sudo systemctl start ssh
sudo systemctl enable ssh
sudo systemctl enable ssh
sudo systemctl status ssh
```

after that there is a raspberry that expose an ssh servie.
we can call thi endpoint in that way

- EdgeUGV, port: 22 ( usr: ws, psw: ws)

.ssh/config

```bash
# v
Host EdgeUGV
    HostName 192.168.0.144
    User ws
    # Optional: Add these for convenience
    Port 22
    IdentityFile ~/.ssh/id_rsa
    ForwardAgent yes
    ServerAliveInterval 60
    ServerAliveCountMax 3
```

Establish a connection to:

```bash
ssh root@192.168.0.144 -p 22
```

## first startup

when you have established the connection with `edgeUGV` endpoint
you need to follow this steps
[UGV Beast PI ROS2 1. Preparation](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2_1._Preparation)
[ugv_ws](https://github.com/waveshareteam/ugv_ws)
Real time display of tasks and process information currently running in the system on the terminal:
Wait a few seconds, and you will see the PID with COMMAND as python.

```bash
top
```

```bash
Kill -9 Python process PID
```

```bash
crontab -e
```

open crontab's configuration file, you can see the following two lines:

```bash
@reboot ~/ugv_pt_rpi/ugv-env/bin/python ~/ugv_pt_rpi/app.py >> ~/ugv.log 2>&1
@reboot /bin/bash ~/ugv_pt_rpi/start_jupyter.sh >> ~/jupyter_log.log 2>&1
```

Add a "#" sign at the beginning of the line "...app.py >> ...." to comment out this line, as follows:

```bash
#@reboot ~/ugv_pt_rpi/ugv-env/bin/python ~/ugv_pt_rpi/app.py >> ~/ugv.log 2>&1
@reboot /bin/bash ~/ugv_pt_rpi/start_jupyter.sh >> ~/jupyter_log.log 2>&1
```

Start Docker remote service

```bash
cd /home/ws/ugv_ws
sudo chmod +x ros2_humble.sh remotessh.sh
```

Execute the script file ros2_humble.sh to start the Docker remote service:

```bash
./ros2_humble.sh
```

## Automation (The "Boot to ROS" Setup)

This is the most important part: making the robot accessible via SSH on Port 23 automatically.

1. Enable Docker Autostart

```bash
docker update --restart unless-stopped ugv_rpi_ros_humble
```

2. Create the SSH Auto-Trigger Service
   Create a system service to start the SSH server inside the container at boot.

```bash

sudo nano /etc/systemd/system/ugv_ssh_start.service
Paste this content:

Ini, TOML

[Unit]
Description=Start SSH inside UGV Docker Container
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/docker exec ugv_rpi_ros_humble service ssh start
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

3. Activate the Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable ugv_ssh_start.service
sudo systemctl start ugv_ssh_start.service
```

---

### ROS2 Hubble Docker container connection

Now our edge have two ssh connection.  
The workstation:

- EdgeUGV, port: 22 ( usr: ws, psw: ws)
- and the EdgeUGV-ros2-docker, port 23 ( usr: root, psw: ws )

```bash
ssh root@192.168.0.144 -p 22
ssh root@192.168.0.144 -p 23
```

Establish a connection to:

```bash
#EdgeUGV
Host EdgeUGV-ros2-docker
    HostName 192.168.0.144
    User root
    # Optional: Add these for convenience
    Port 23
    IdentityFile ~/.ssh/id_rsa
    ForwardAgent yes
    ServerAliveInterval 60
    ServerAliveCountMax 3
```

using

```bash
ssh root@192.168.0.144 -p 23
```

## first UGV Beast Hubble setup

```bash
cd ~/ugv_ws

chmod +x build_first.sh
./build_first.sh

# Now rebuild from scratch
colcon build --symlink-install
```

```bash
cd ugv_ws
```

unzip the description file

```bash
file /root/ugv_ws/ugv_description.zip
or
unzip ugv_description.zip -d src/
```

```bash
source /root/ugv_ws/install/setup.bash
echo "source /root/ugv_ws/install/setup.bash" >> ~/.bashrc
```

```bash
sudo locale-gen en_US.UTF-8 && export LANG=en_US.UTF-8
export LANG=en_US.UTF-8 && export LC_ALL=en_US.UTF-8 && locale
```

## UGV Beast custom configurato to work with Cyberwave

to do that I forked the official repo in order to have our fork

[Cyberwave UGV Beast](https://github.com/cyberwave-os/ugv_ws/tree/ros2-humble-develop)

To set up a new UGV that already has the official Waveshare code, you don't need to delete everything and start over. You can simply "point" the existing workspace to your custom fork and then use the `vcstool` to fill in the missing libraries.

Here is the exact procedure to run on the new UGV:

### 1. Link the New UGV to Your Fork

Go into the existing workspace and add your repository as a new remote:

```bash
cd ~/ugv_ws
# Add your GitHub as a remote called 'origin_custom' (or rename existing ones)
git remote add cyberwave https://github.com/cyberwave-os/ugv_ws.git
git fetch cyberwave
```

### 2. Switch to Your Custom Branch

This will replace the official Waveshare files with your custom `ugv_main` and the multi-repo configuration:

```bash
# Force switch to your beast-dev branch
git checkout -B beast-dev cyberwave/beast-dev
```

### 3. Clean Up & Install External Libraries

Because your new branch uses `workspace.repos`, the official libraries (like `navigation2`) that were previously part of the main repo are now untracked or missing. Clean them up and re-download them correctly:

```bash
# 1. (Optional but recommended) Remove the old official library folders to ensure a clean start
# These are the ones now managed by workspace.repos
rm -rf src/navigation2 src/BehaviorTree.CPP src/bond_core src/test_interface_files src/vision_opencv

# 2. Use the strategy to download the correct versions of all libraries
vcs import src < workspace.repos
```

### 4. Install System Tools & Dependencies

Since this is a new UGV, you need to make sure all system-level tools (`vcstool`, `rosdep`) and binary dependencies are installed:

```bash
# Install vcstool if not present
sudo apt update && sudo apt install python3-vcstool -y

# Install all ROS dependencies required by your custom code
sudo rosdep init  # Only if never run before
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### 5. Build the Workspace

Now you can build your custom beast configuration:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

### Summary of what just happened:

1.  **Code Replacement**: `git checkout` replaced the Waveshare drivers/logic with your custom versions.
2.  **Modularization**: `vcs import` downloaded the heavy standard libraries (`navigation2`, etc.) into the `src/` folder, but they are kept separate from your git history.
3.  **Tooling**: `rosdep` ensured that any extra Linux packages needed by your custom code are installed.

**From now on, on this new UGV, you just use the standard `git pull cyberwave beast-dev` to get your latest updates.**

This documentation explains the **Unified Architecture** used for the UGV Beast and provides a step-by-step guide to replicate this configuration from a standard Waveshare repository.

---

# Documentation: Unified Architecture for UGV Beast

## 1. Why this modification is necessary

The standard Waveshare repository is designed for generic robots and uses a "Split Node" architecture. However, the **UGV Beast** requires a **Unified Node** (`ugv_bringup.py`) for the following technical reasons:

### A. Serial Port Exclusivity (The Critical Reason)

The Raspberry Pi communicates with the ESP32 sub-controller via a single physical UART line (`/dev/ttyAMA0`).

- **The Conflict**: In Linux, a serial port should only be opened by **one process** at a time.
- **The Result of "Split" Nodes**: If `ugv_bringup` (Sensors) and `ugv_driver` (Commands) both try to open the same port, the second node will fail with a "Device or Resource Busy" error, or they will corrupt each other's data packets.
- **The Solution**: By unifying all code into one node, we ensure a single, stable connection to the hardware.

### B. Path Alignment for Root Users

The official code hardcodes `/home/ws/ugv_ws/`. Since your environment runs as `root` in `/root/ugv_ws/`, the official code would crash immediately when trying to find the `low_battery.wav` file or configuration parameters.

### C. Low Battery Alert Reliability

In the official setup, the audio alert is in the `ugv_driver` node. Since we disable that node to prevent serial conflicts, we must move the audio logic into the unified node so the robot can still warn you when the battery is low.

---

## 2. Technical Feature Summary

| Feature                | Official Implementation      | Unified Implementation               |
| :--------------------- | :--------------------------- | :----------------------------------- |
| **Node Structure**     | Two nodes (Bringup + Driver) | **One node (Unified Bringup)**       |
| **Serial Connection**  | Potential collisions         | **Thread-safe exclusive access**     |
| **Command Processing** | Handled in `ugv_driver.py`   | **Integrated into `ugv_bringup.py`** |
| **Decoding**           | Standard UTF-8               | **Standard UTF-8**                   |
| **File Paths**         | `/home/ws/ugv_ws/`           | **`/root/ugv_ws/`**                  |

---

## 3. How to Replicate this Configuration

If you ever need to recreate this setup from a fresh Waveshare repository, follow these four steps:

### Step 1: Modify `ugv_bringup.py` Imports

Add the necessary imports for system commands and audio at the top of the file:

```python
import subprocess
# ... other official imports ...
```

### Step 2: Add Command Subscribers

In the `__init__` method of the `ugv_bringup` class, add the subscribers that are officially found in `ugv_driver.py`:

```python
self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)
self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)
```

### Step 3: Implement the Callbacks

Copy the logic for `cmd_vel_callback`, `joint_states_callback`, and `led_ctrl_callback` from `ugv_driver.py` into `ugv_bringup.py`. Ensure they use `self.base_controller.send_command(data)` to send data to the serial port.

### Step 4: Integrate the Audio Alert

Inside the `publish_voltage` function, add the logic to check the voltage and trigger `aplay`:

```python
if 0.1 < voltage_value < 9:
    try:
        subprocess.run(['aplay', '-D', 'plughw:3,0', '/root/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
        time.sleep(5)
    except Exception:
        pass
```

### Step 5: Build and Clean

Always perform a clean build to ensure the new architecture is recognized:

```bash
cd /root/ugv_ws
rm -rf build/ugv_bringup install/ugv_bringup
colcon build --packages-select ugv_bringup --symlink-install
source install/setup.bash
```

### Step 6: Launch

Always use the Beast-specific launch file, which is designed to work with this unified node:

```bash
ros2 launch ugv_bringup master_beast.launch.py
```

### Build MQTT_bridge

```bash
cd /root/ugv_ws/src/
git clone <mqtt_bridge>
```

```bash
# Navigate to workspace root
cd /root/ugv_ws

# Source the ROS 2 installation (IMPORTANT!)
source /opt/ros/humble/setup.bash
source install/setup.bash
#Clean the package:
rm -rf install/mqtt_bridge build/mqtt_bridge

#if necesesary
 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build the package
colcon build --packages-select mqtt_bridge

# Source the workspace overlay
source /root/ugv_ws/install/setup.bash
source install/setup.bash
```

Here are the proper commands to build the MQTT bridge:

the first time you need to install cyberwave-python sdk

```bash
pip install cyberwave
```

### Launch MQTT Bridge

```bash
# Launch with robot configuration
ros2 launch mqtt_bridge mqtt_bridge.launch.py robot_id:=robot_ugv_beast_v1
```

## Re-Build the packege after some change

```bash
# 1. Rebuild with the fix
cd /root/ugv_ws
rm -rf install/mqtt_bridge build/mqtt_bridge
colcon build --packages-select mqtt_bridge --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# 2. Launch the bridge
ros2 launch mqtt_bridge mqtt_bridge.launch.py robot_id:=robot_ugv_beast_v1
or
ros2 run mqtt_bridge mqtt_bridge_node --ros-args --params-file /root/ugv_ws/install/mqtt_bridge/share/mqtt_bridge/config/params.yaml -p robot_id:=robot_ugv_beast_v1
```

---

## Start the rover control

Workspace Initialization

## Launch the Core System

To start the Lidar, Robot State, and Driver all at once:

```bash
# Ensure models are set (if not in .bashrc)
export UGV_MODEL=ugv_beast
echo $UGV_MODEL
export LDLIDAR_MODEL=ld19
```

```bash
cd ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ugv_bringup ugv_driver
```

```bash
# starts the serial bridge between the Raspberry Pi and the ESP32. used only for isolated testing of motor commands or LED controls when no other systems (like LiDAR or Navigation)
ros2 run ugv_bringup ugv_driver


# starts Laser scanner driver. This is necessary for SLAM, Navigation, and monitoring battery/joint positions.
ros2 launch ugv_bringup bringup_lidar.launch.py


# A dedicated launch file for the vision system. Activates the CSI or USB camera driver and the image processing pipeline.This usually runs on a separate interface (USB or MIPI CSI) and rarely conflicts with the serial port.
ros2 launch ugv_vision camera.launch.py
```

Launch the Bridge\*\*

Launch the MQTT Bridge (For Remote Control)
Open a **second terminal** and run:

```bash
# Terminal 1 - Launch bridge
cd ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mqtt_bridge mqtt_bridge.launch.py robot_id:=robot_ugv_beast_v1
```

---

## Automation & Helper Scripts

create the master_beast.launch.py to run all the drivers with one single cli command

create that file into
/root/ugv_ws/src/ugv_main/ugv_bringup/launch/master_beast.launch.py

### how to create the `master_beast.launch.py` file

This file is the "brain" of your robot's software startup, merging hardware drivers, sensor fusion (EKF), cloud connectivity (MQTT), and vision into a single command.

### **Step 1: Understand the Goal**

The `master_beast.launch.py` simplifies the startup process by:

1.  **Starting the Base Hardware**: IMU, Motors, and Lidar.
2.  **Enabling EKF Fusion**: Fusing IMU and Encoders for better accuracy.
3.  **Managing Transforms (TF)**: Turning off the basic driver transform so the EKF can take over.
4.  **Connecting to Cloud**: Launching the MQTT bridge.
5.  **Starting Vision**: Initializing the camera stream.

---

### **Step 2: Create the File**

Create the file at `~/ugv_ws/src/ugv_main/ugv_bringup/launch/master_beast.launch.py` and paste the following code:

you can find that file into: /mqtt_bridge/scripts/ugv_beast/master_beast.launch.py

```python
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to packages and configurations
    ugv_bringup_dir = get_package_share_directory('ugv_bringup')
    ugv_vision_dir = get_package_share_directory('ugv_vision')
    ugv_description_dir = get_package_share_directory('ugv_description')
    mqtt_bridge_dir = get_package_share_directory('mqtt_bridge')
    ldlidar_dir = get_package_share_directory('ldlidar')

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

    # 3. Core Hardware Node (Unified Bringup)
    # Handles Serial communication for Telemetry and Commands
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        name='ugv_bringup',
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('ugv/pt_ctrl', '/ugv/pt_ctrl'),
            ('ugv/led_ctrl', '/ugv/led_ctrl'),
            ('voltage', '/voltage'),
            ('imu/data_raw', '/imu/data_raw'),
            ('imu/mag', '/imu/mag'),
            ('odom/odom_raw', '/odom/odom_raw'),
        ]
    )

    # 4. IMU Filtering
    # Processes raw IMU data into a stable orientation
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ]
    )

    # 5. Lidar Driver
    # Includes the dedicated lidar launch file
    from launch.conditions import IfCondition
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_dir, 'launch', 'ldlidar.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )

    # 6. Robot Description & Transforms
    # Publishes the 3D model and static transforms
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_description_dir, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # 7. Odometry Calculator
    # Computes raw odometry from wheel encoders
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        name='base_node',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        remappings=[
            ('imu/data', '/imu/data'),
            ('odom/odom_raw', '/odom/odom_raw'),
            ('odom', '/odom')
        ]
    )

    # 8. Cloud Connectivity (MQTT Bridge)
    mqtt_bridge_node = Node(
        package='mqtt_bridge',
        executable='mqtt_bridge_node',
        name='mqtt_bridge_node',
        parameters=[mqtt_config_path, {'robot_id': LaunchConfiguration('robot_id')}],
        output='screen'
    )

    # 9. Video Streaming (Camera)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_vision_dir, 'launch', 'camera.launch.py')
        )
    )

    return LaunchDescription([
        pub_odom_tf_arg,
        robot_id_arg,
        use_lidar_arg,
        bringup_node,
        imu_filter_node,
        laser_launch,
        robot_state_launch,
        base_node,
        mqtt_bridge_node,
        camera_launch
    ])

```

---

### **Step 3: Key Logic Explained**

- **`pub_odom_tf='false'`**: This is the most important part. By default, the robot driver tries to say "I am here" based on wheel counts. The EKF says "I am here" based on wheels + IMU. If both talk at once, the robot "ghosts" or flickers in RViz. We force the driver to be quiet so EKF can be the source of truth.
- **`IncludeLaunchDescription`**: Instead of rewriting the code for the camera or lidar, we simply "import" their existing launch files.
- **`Node(...)`**: We manually define the EKF and MQTT nodes because they require specific configuration files (`ekf.yaml` and `params.yaml`).

---

### **Step 4: Build and Execute**

After saving the file, you must tell ROS that a new launch file exists by rebuilding the package:

```bash
# 1. Build
cd ~/ugv_ws
colcon build --packages-select ugv_bringup

# 2. Update Environment
source install/setup.bash

# 3. Run
ros2 launch ugv_bringup master_beast.launch.py
```

```bash
# Start the complete UGV Beast stack with one command
ros2 launch ugv_bringup master_beast.launch.py
or
ros2 launch ugv_bringup master_beast.launch.py robot_id:=robot_ugv_beast_v1

# Or with custom robot ID
ros2 launch /root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/master_beast.launch.py robot_id:=your_robot_id_here
```

## Other UGV Beast Helper Scripts for development

### 1. Full Environment Startup

Automatically cleans up existing processes and starts the entire UGV stack (Bringup, Driver, Vision, BaseNode, and MQTT Bridge) in a managed `tmux` session.
cd /root/ugv_ws
./src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh --logs

### 2. MQTT Bridge Clean Build & Run

Removes existing build/install artifacts and performs a fresh `colcon build`.
cd /root/ugv_ws
./src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh --logs

# Run UGV Edge in Cyberwave

This guide explains how to run the UGV Beast edge processes to enable video streaming, teleoperation, and digital twin updates on the Cyberwave platform.

## 1. Environment Configuration

Set your Cyberwave credentials. These are used by the `mqtt_bridge` to authenticate with the platform.

```bash
export CYBERWAVE_TOKEN="your-token-here"
export CYBERWAVE_TWIN_UUID="your-twin-uuid-here"
export CYBERWAVE_ENVIRONMENT="production"
```

Set Your Twin UUID (REQUIRED)

```bash
# Edit the file
nano /opt/ros/humble/src/mqtt_bridge/config/mappings/robot_rower_v1.yaml

# Change this:
twin_uuid: "00000000-0000-0000-0000-000000000000"

# To your actual UUID:
twin_uuid: "YOUR-ACTUAL-UUID-FROM-CYBERWAVE-PLATFORM"
```

- **CYBERWAVE_TOKEN**: Your API token for the platform.
- **CYBERWAVE_TWIN_UUID**: The unique identifier for your robot's digital twin.

## 2. ROS 2 Setup

Source the ROS 2 environment and your local workspace.

Build `mqtt_bridge`

```bash
# Build
cd /opt/ros/humble
colcon build --packages-select mqtt_bridge
```

```bash
export UGV_MODEL=ugv_beast
source /opt/ros/humble/setup.bash
source install/setup.bash
```

then if you already create the master beast launcher

```bash
# Start the complete UGV Beast stack with one command
ros2 launch ugv_bringup master_beast.launch.py
or
ros2 launch ugv_bringup master_beast.launch.py robot_id:=robot_ugv_beast_v1
```

## Troubleshooting

If the stream does not start or commands are not responding, try cleaning up all processes and restarting:

```bash
sudo pkill -9 -f mqtt_bridge_node
sudo pkill -9 -f ugv_driver
sudo pkill -9 -f ugv_bringup
sudo pkill -9 -f usb_cam
sudo pkill -9 -f wheel_joint_publisher
```

## ROS command-line tools instead\*\*

Since you're working with robotics on a headless Raspberry Pi, you can use command-line tools to monitor your system:

to get most of this ros2 information you should run

```bash
ros2 launch ugv_bringup bringup_lidar.launch.py
```

```bash
# Monitor topics
ros2 topic list
ros2 topic echo /cmd_vel

# Monitor nodes
ros2 node list
ros2 node info /joy_ctrl

# Use rqt without GUI
ros2 run rqt_graph rqt_graph  # (still needs display)
```

```bash
source ~/ugv_ws/install/setup.bash && ros2 node info /ugv_bringup
```

## Show joints state

```bash
ros2 topic echo /ugv/joint_states --once

ros2 interface show sensor_msgs/msg/JointState

ros2 node info /ugv_driver
```

# View camera information

```bash
sudo apt-get install -y v4l-utils
```

## Check Camera Info (Command Line)

```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext

source ~/ugv_ws/install/setup.bash && ros2 launch ugv_vision camera.launch.py &
```

```bash
ros2 topic info /image_raw
```

```bash
ros2 topic echo /camera_info --once

# Check image publish rate
ros2 topic hz /image_raw

# Check topic info
ros2 topic info /image_raw
```

Option 4: Save Images to File

```bash
ros2 run image_view image_saver --ros-args -r image:=/image_raw
# Save a single image
ros2 run image_view image_saver --ros-args -r image:=/image_raw
```

```bash
source /root/ugv_ws/install/setup.bash
ros2 launch mqtt_bridge mqtt_bridge.launch.py robot_id:=robot_ugv_beast_v1
ros2 node info /ugv_driver

ros2 topic list && ros2 node list

ros2 topic hz /ugv/led_ctrl & ros2 topic hz /cmd_vel & ros2 topic hz /ugv/joint_states

ros2 topic info /ugv/joint_states --verbose

ros2 topic echo /voltage
```

> **MQTT auth for the examples below:** the broker username is the public
> default `mqttcyb`; the password is **your Cyberwave API key**. Never hardcode
> it — export it once and the `-P "$CYBERWAVE_API_KEY"` examples will pick it up:
>
> ```bash
> export CYBERWAVE_API_KEY="cw_…"   # your token; do not commit
> ```

## **3. Test Headlights (LED Control)**

```bash
# Set your UUID first
export TWIN_UUID="00000000-0000-0000-0000-000000000000"  # Replace with yours

# Monitor LED topic (Terminal 3)
ros2 topic echo /ugv/led_ctrl

sudo apt update
sudo apt install -y mosquitto-clients

# Turn all lights ON (Terminal 2)
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"lights","data":{"all":255}}'

# Wait 3 seconds, then turn OFF
sleep 3
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"lights","data":{"all":0}}'

# Test chassis light only
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"lights","data":{"chassis_light":255,"camera_light":0}}'
```

## **4. Test Velocity Control**

Movement uses the `velocity_command` payload (the legacy `cmd_vel` command is
removed). `linear_x`/`angular_z` are clamped to `CYBERWAVE_UGV_MAX_LINEAR_SPEED`
/ `CYBERWAVE_UGV_MAX_ANGULAR_SPEED` (defaults 0.8 m/s / 1.0 rad/s). `duration_ms`
arms a deadman stop, so resend while driving.

```bash
# Monitor velocity commands (Terminal 3)
ros2 topic echo /cmd_vel

# Send forward motion (Terminal 2)
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"velocity_command","source_type":"tele","velocity_command":{"linear_x":0.3,"angular_z":0,"duration_ms":600}}'

# Stop
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"stop","source_type":"tele"}'

# Turn in place
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"velocity_command","source_type":"tele","velocity_command":{"linear_x":0,"angular_z":0.5,"duration_ms":600}}'
```

## **5. Test Pan-Tilt Camera**

```bash
# Monitor joint states (Terminal 3)
ros2 topic echo /ugv/joint_states

# Move camera (Terminal 2)
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"pan_tilt","data":{"pan":0.5,"tilt":0.3}}'

# Center camera
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"pan_tilt","data":{"pan":0,"tilt":0}}'
```

## **6. Test Emergency Stop**

```bash
# Monitor emergency stop (Terminal 3)
ros2 topic echo /emergency_stop

# Activate E-stop (Terminal 2)
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"estop","data":{"activate":true}}'

# Deactivate E-stop
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"estop","data":{"activate":false}}'
```

## **9. Direct ROS Test (No MQTT)**

```bash
# Test without MQTT - publish directly to ROS topics

# Test headlights
ros2 topic pub /ugv/led_ctrl std_msgs/msg/Float32MultiArray "{data: [255, 255]}" -1

# Test velocity
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" -1

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" -1
```

## **Quick Test Sequence (Copy-Paste)**

```bash
# Replace with your actual UUID!
export TWIN_UUID="00000000-0000-0000-0000-000000000000"

# Test 1: Lights ON
echo "Testing: Lights ON"
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "cyberwave/twin/$TWIN_UUID/command" -m '{"command":"lights","data":{"all":255}}'
sleep 2

# Test 2: Lights OFF
echo "Testing: Lights OFF"
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "cyberwave/twin/$TWIN_UUID/command" -m '{"command":"lights","data":{"all":0}}'
sleep 2

# Test 3: Move forward slowly
echo "Testing: Move forward"
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "cyberwave/twin/$TWIN_UUID/command" -m '{"command":"velocity_command","source_type":"tele","velocity_command":{"linear_x":0.2,"angular_z":0,"duration_ms":2500}}'
sleep 2

# Test 4: Stop
echo "Testing: Stop"
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "cyberwave/twin/$TWIN_UUID/command" -m '{"command":"stop","source_type":"tele"}'

echo "Tests complete!"
```

**Monitor all UGV topics at once:**

```bash
ros2 topic echo /ugv/led_ctrl & \
ros2 topic echo /cmd_vel & \
ros2 topic echo /ugv/joint_states
```

```bash

```

## Control from MQTT

```bash
 mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "localcyberwave/joint/60d542ec-7d65-48b7-83dc-ecd89adbacec/update" -m '{
  "source_type": "tele",
  "joint_names": ["left_up_wheel_link_joint"],
  "points": [
    {
      "positions": [0.5],
      "velocities": [0.0],
      "time_from_start": {
        "sec": 0,
        "nanosec": 100000000
      }
    }
  ]
}'
```

```bash
 mosquitto_sub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "localcyberwave/joint/60d542ec-7d65-48b7-83dc-ecd89adbacec/update" -v
```

## chech the messages

To retrieve the **Commands** (Downstream) and the **Upstream Traffic** (Odometry and Joint States) for your specific twin, use the following `mosquitto_sub` commands.

These commands assume your environment prefix is `local` (as seen in your logs).

### 1. Retrieve Downstream Commands (What the frontend sends)

This will show the `cmd_vel` and `led_ctrl` messages sent from your keyboard:

```bash
mosquitto_sub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "localcyberwave/twin/ace84397-be7d-4f4d-9f66-74217b2f3509/command" -v
```

### 2. Retrieve Upstream Odometry (The physical robot's position)

This will show the `position` and `rotation` updates sent from the physical robot to update the digital twin:

```bash
mosquitto_sub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "localcyberwave/twin/ace84397-be7d-4f4d-9f66-74217b2f3509/status/odom" -v
```

### 3. Retrieve Upstream Joint States (Wheel and Pan-Tilt rotation)

This shows the status of individual joints (e.g., wheel rotation):

```bash
mosquitto_sub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "localcyberwave/joint/ace84397-be7d-4f4d-9f66-74217b2f3509/update" -v
```

### 4. Spy on ALL traffic for this Twin (Recommended)

This wildcard command will catch **every** message associated with this twin across both `twin` and `joint` scopes:

```bash
mosquitto_sub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" -t "localcyberwave/+/ace84397-be7d-4f4d-9f66-74217b2f3509/#" -v
```

### Summary Table for UUID `ace84...509`:

| Scope        | Direction  | MQTT Topic                                                             |
| :----------- | :--------- | :--------------------------------------------------------------------- |
| **Command**  | Downstream | `localcyberwave/twin/ace84397-be7d-4f4d-9f66-74217b2f3509/command`     |
| **Odometry** | Upstream   | `localcyberwave/twin/ace84397-be7d-4f4d-9f66-74217b2f3509/status/odom` |
| **Joints**   | Upstream   | `localcyberwave/joint/ace84397-be7d-4f4d-9f66-74217b2f3509/update`     |

### 📝 Troubleshooting Documentation: Serial Port Resource Conflict

---

#### 🚩 Symptoms

The following errors were observed in the terminal logs:

1.  **JSON Decode Errors:** `JSON decode error: Expecting property name enclosed in double quotes` or `Extra data`.
2.  **Hardware Failures:** `[base_ctrl.feedback_data] unexpected error: device reports readiness to read but returned no data`.
3.  **Python Crashes:**
    ```python
    File "/root/ugv_ws/build/ugv_bringup/ugv_bringup/ugv_bringup.py", line 125, in publish_imu_data_raw
    msg.angular_velocity.x = 3.1415926 * float(imu_raw_data["gx"]) / (16.4 * 180)
    KeyError: 'gx'
    ```
4.  **Control Failure:** MQTT commands to `/cmd_vel` were ignored or intermittent.

---

#### 🔍 Root Cause: Serial Port Contention

The problem was caused by **Resource Contention** on the serial port `/dev/ttyAMA0`.

Two separate processes were attempting to communicate with the robot hardware simultaneously:

1.  `ros2 run ugv_bringup ugv_driver` (Manual Node)
2.  `ros2 launch ugv_bringup bringup_lidar.launch.py` (Launch File)

Because `bringup_lidar.launch.py` **already includes** the `ugv_driver` node, running both commands meant two programs were "fighting" for the same physical wire. This resulted in:

- **Data Fragmentation:** Bits of data were being "stolen" by one process from the other.
- **Mangled JSON:** The JSON strings sent by the robot were cut in half, making them unreadable.
- **Missing Keys:** When a process received a partial message, essential data like `"gx"` (gyroscope X-axis) was missing, leading to the Python `KeyError`.

---

#### ✅ Solution

The conflict was resolved by **terminating the duplicate process**.

**Correct Procedure:**

1.  Stop all running ROS nodes.
2.  Check for background "zombie" processes: `ps aux | grep python`.
3.  Launch the system using **only** the combined launch file:
    ```bash
    ros2 launch ugv_bringup bringup_lidar.launch.py
    ```

---

**1. Clean the environment variables** (this wipes the "memory" of your native build):

```bash
unset AMENT_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
# Reset your PATH to system defaults
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

```

**2. Source ONLY the internal Docker ROS 2:**

```bash
source /opt/ros/humble/setup.bash

```

**3. Verify immediately:**

```bash
which ros2

```

- **Correct Output:** `/opt/ros/humble/bin/ros2`
- **If you see this, proceed to step 4.**

**4. Source your UGV workspace (Carefully):**

```bash
source /home/ws/ugv_ws/install/setup.bash

```

###🚀 Try the launch againNow that `which ros2` points to `/opt/ros/humble/bin/ros2`, try the UGV tool again:

```bash
ros2 launch ugv_description display.launch.py use_rviz:=false

```

## Build the Cyberwave UGV controller locally

In order to be able to build and test a new controller locally
go into `cyberwave-backend/src/app/management/commands/seed_controllers.py`
set you local endpoint as `MAIN_URL` variable (row 30)
set your local CYBERWAVE_TOKEN (row 88)

run you Backend

```bash
cd cyberwave/cyberwave-backend
docker compose up
```

then you can build it with

```bash
cd cyberwave/cyberwave-backend
source .venv/bin/activate
python manage.py seed_controllers
```

set the ENV variables

```bash
export CYBERWAVE_TWIN_UUID=
export CYBERWAVE_TOKEN=
```

go in cyberwave.com > catalog > UGV Beast > Edit Asset > activate `can_locomote` and `has_joints` > Save Chnages

go into the UGV beast evironment > add UGB Digita;l Twin > click over the asset > `Assign Controller` > chose the controller you developed

To achieve a "manual mode" where you get **Joint Updates** and **Odometry** without using the full `bringup_lidar.launch.py` (and thus avoiding the Lidar and serial contention), you need to run the **core hardware interface** nodes yourself.

### 1. The "Manual Mode" Recipe

When you run a launch file like `bringup_lidar.launch.py`, it essentially "wraps" several independent nodes together. To get just the movement data (Joints + Odom) manually, you must run these two specific components:

#### Step 1: Start the Driver (The Hardware Link)

This node establishes the serial connection to the ESP32.

```bash
export UGV_MODEL=ugv_beast
export LDLIDAR_MODEL=ld19
source /root/ugv_ws/install/setup.bash
ros2 run ugv_bringup ugv_driver

```

- **What it does:** It opens `/dev/ttyAMA0`, sends motor commands, and reads encoder/IMU data.

#### Step 2: Start the State Publisher (The Coordinate Link)

The driver sends raw data, but ROS 2 needs a "State Publisher" to turn those numbers into a 3D coordinate system (TF Tree) and formatted `/joint_states`.
In another terminal, run:

```bash
ros2 launch ugv_description description.launch.py

```

- **What it does:** It reads the UGV Beast URDF (robot model) and publishes the relationship between the wheels, the chassis, and the sensors. Without this, your odometry will have no "base" to attach to.

---

### 2. Detailed Configuration Analysis

By running the driver manually, you are using the **Base Configuration**. Here is exactly what is happening under the hood:

| Component                | Manual Command          | Role in "Manual Mode"                                                                         |
| ------------------------ | ----------------------- | --------------------------------------------------------------------------------------------- |
| **Serial Communication** | `ugv_driver`            | Polls the ESP32 at ~10-20Hz. If it crashes, you'll see the `KeyError` or "Mangled JSON" here. |
| **Odometry Publisher**   | Part of `ugv_driver`    | Calculates distance based on ticks \times wheel_circumference. Published to `/odom`.          |
| **Joint Broadcaster**    | `description.launch.py` | Takes the raw encoder ticks and updates the rotation of `left_up_wheel_link_joint`, etc.      |
| **Transform (TF)**       | `robot_state_publisher` | Broadcats the `odom` \rightarrow `base_footprint` transform.                                  |

---

### 3. How to Verify "Manual Mode" is Working

Once you have the two commands above running, use these three "Health Check" commands:

1. **Check the Joints:**
   `ros2 topic echo /joint_states`

- _Expected:_ You should see 4 wheel joints and 2 pan-tilt joints updating as you move the robot.

2. **Check the Odometry:**
   `ros2 topic echo /odom --field pose.pose.position`

- _Expected:_ X and Y values should increase/decrease when you push the robot.

3. **Check the Linkage (TF):**
   `ros2 run tf2_ros tf2_echo odom base_footprint`

- _Expected:_ A continuous stream of Translation and Rotation data.

### 4. Why this is safer than the Launch File

By avoiding `bringup_lidar.launch.py`, you are **disabling the Lidar node**. Since the Lidar often uses a high data rate or shares power/serial resources on some Raspberry Pi HATs, disabling it prevents the "Serial Port Contention" you diagnosed. This allows the `ugv_driver` to have a clean, uninterrupted conversation with the ESP32.

**Would you like me to create a "Minimum-Movement" launch file for you that only starts the driver and the state publisher, but leaves the Lidar off?**

root@raspberrypi:~/ugv_ws# sudo chmod 777 /dev/ttyS0
root@raspberrypi:~/ugv_ws# unset FASTRTPS_DEFAULT_PROFILES_FILE
root@raspberrypi:~/ugv_ws# ros2 run ugv_bringup ugv_driver
