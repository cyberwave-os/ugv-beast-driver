import typing
from typing import Any, Dict, List, Optional
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .plugins.internal_odometry import InternalOdometry

class TelemetryProcessor:
    def __init__(self, node: Node, internal_odom: Optional[InternalOdometry] = None):
        self.node = node
        self._internal_odom = internal_odom
        self._joint_state_initialized = False

    def process_joint_states(self, msg: JointState) -> None:
        if not hasattr(self.node, '_mapping') or self.node._mapping is None:
            return

        try:
            mapping = self.node._mapping
            state_key = 'trajectory_accumulated_state'
            
            # Get existing accumulated state if available to preserve values not in this update
            if not hasattr(self.node, '_accumulated_joint_states'):
                self.node._accumulated_joint_states = {}
            
            existing_accumulated = self.node._accumulated_joint_states.get('initial_joint_state')
            if existing_accumulated and len(existing_accumulated) == len(mapping.joint_names):
                accumulated = list(existing_accumulated)
            else:
                accumulated = [0.0] * len(mapping.joint_names)
            
            name_to_pos = {}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    name_to_pos[name] = msg.position[i]
            
            for idx, ros_name in enumerate(mapping.joint_names):
                is_virtual = False
                io_config = getattr(self.node, '_io_config', {})
                for tool_name, config in io_config.items():
                    if ros_name == config.get('joint_name'):
                        accumulated[idx] = config.get('off_value', 0.0)
                        is_virtual = True
                        break
                
                if not is_virtual and ros_name in name_to_pos:
                    accumulated[idx] = name_to_pos[ros_name]
            
            if not hasattr(self.node, '_accumulated_joint_states'):
                self.node._accumulated_joint_states = {}
            self.node._accumulated_joint_states['initial_joint_state'] = list(accumulated)
            
            self.node._accumulated_joint_states[state_key] = list(accumulated)
            
            if not self._joint_state_initialized:
                self._joint_state_initialized = True
                self.node.get_logger().info("Initialized accumulated joint state from feedback")

            if self._internal_odom:
                self._internal_odom.update(msg)
                
        except Exception as e:
            self.node.get_logger().error(f"Failed to process joint states: {e}")
