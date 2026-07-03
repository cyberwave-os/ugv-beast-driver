from setuptools import setup

package_name = 'mqtt_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        package_name + '.plugins'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/config", ['config/params.yaml']),
        ("share/" + package_name + "/config/mappings", ['config/mappings/default.yaml', 'config/mappings/robot_ur7_v1.yaml', 'config/mappings/robot_ugv_beast_v1.yaml']),
        ("share/" + package_name + "/launch", ['launch/bridge_launch.py', 'launch/mqtt_bridge.launch.py'])
    ],
    install_requires=['setuptools', 'cyberwave==0.5.2', 'psutil'],
    zip_safe=True,
    maintainer='Cyberwave',
    maintainer_email='info@cyberwave.com',
    description='Simple ROS2 <-> MQTT bridge (inspired by mqtt_client)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = mqtt_bridge.mqtt_bridge_node:main'
        ],
    },
)
