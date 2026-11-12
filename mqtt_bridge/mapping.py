"""Mapping loader and remap helpers for joint mappings.

This module implements a small Mapping class that loads a per-robot YAML
mapping file (json_by_name format) and provides fast remap helpers used by
the MQTT bridge for ros<->mqtt translations of JointState messages.

Design goals:
- Keep mapping files human-editable and versioned under config/mappings/
- Precompute per-joint transforms and name aliases for fast per-message
  conversion.
- Provide a reload() method so the bridge can hot-reload mappings on demand.
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Dict, List, Optional

import yaml
from sensor_msgs.msg import JointState

# Import source type constants from Cyberwave SDK (with fallback)
try:
    from cyberwave import SOURCE_TYPE_EDGE, SOURCE_TYPE_TELE, SOURCE_TYPE_EDIT, SOURCE_TYPE_SIM  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    # Fallback constants if SDK is not available
    SOURCE_TYPE_EDGE = 'edge'  # Upstream: Messages FROM physical robot/edge device
    SOURCE_TYPE_TELE = 'tele'  # Downstream: Teleoperation commands TO robot
    SOURCE_TYPE_EDIT = 'edit'  # Future: Trajectory editor commands
    SOURCE_TYPE_SIM = 'sim'    # Future: Simulation state updates


def _make_transform_fns(spec: Dict[str, Any]):
    # spec may include: scale (float), offset (float), invert (bool)
    scale = float(spec.get('scale', 1.0)) if spec else 1.0
    offset = float(spec.get('offset', 0.0)) if spec else 0.0
    invert = bool(spec.get('invert', False)) if spec else False

    def forward(x):
        try:
            v = float(x)
        except Exception:
            return float('nan')
        if invert:
            v = -v
        v = v * scale + offset
        return v

    def reverse(x):
        try:
            v = float(x)
        except Exception:
            return float('nan')
        # inverse of: v_out = (invert? -v_in : v_in) * scale + offset
        try:
            v = (v - offset) / scale if scale != 0 else float('nan')
        except Exception:
            return float('nan')
        if invert:
            v = -v
        return v

    return forward, reverse


class Mapping:
    """Represent a loaded mapping for one robot.

    Currently supports only `json_by_name` format (recommended). The mapping
    file provides a list of joints and optional transforms/aliases.
    """

    def __init__(self, path: str):
        self.path = path
        self.loaded_at = 0.0
        self.raw: Dict[str, Any] = {}
        self.format = 'json_by_name'
        # optional twin uuid from mapping metadata
        self.twin_uuid: Optional[str] = None
        # ordered list of ros joint names expected by this robot mapping
        self.joint_names: List[str] = []
        # ros_name -> mqtt_name mapping (mqtt_name defaults to same as ros_name)
        self.name_to_mqtt: Dict[str, str] = {}
        # mqtt_name -> ros_name (inverse)
        self.mqtt_to_name: Dict[str, str] = {}
        # per-joint transform functions
        self.transforms: Dict[str, Any] = {}

        # load immediately
        self.reload()

    def reload(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError(self.path)
        with open(self.path, 'r') as f:
            doc = yaml.safe_load(f) or {}
        self.raw = doc
        # read optional twin uuid from top-level or metadata.
        self.twin_uuid = doc.get('twin_uuid') or (doc.get('metadata') or {}).get('twin_uuid')
        self.format = doc.get('format', 'json_by_name')

        if self.format != 'json_by_name':
            raise ValueError(f"Unsupported mapping format: {self.format}")

        joints = doc.get('joints', []) or []
        self.joint_names = []
        self.name_to_mqtt = {}
        self.mqtt_to_name = {}
        self.transforms = {}
        # reverse transforms for mqtt->ros
        self.reverse_transforms = {}
        
        # Load robot constants for trajectory time calculation
        self.robot_constants: Dict[str, Any] = doc.get('robot_constants', {}) or {}

        # Load capabilities
        capabilities = doc.get('capabilities', {}) or {}
        self.supports_atomic_pose = bool(capabilities.get('supports_atomic_pose', False))
        self.upstream_mode = capabilities.get('upstream_mode', 'joint') # 'joint', 'pose', or 'both'
        self.upstream_topics_list = capabilities.get('upstream_topics', ['joint'])

        # Load dynamic command registry if specified
        self.command_registry = doc.get('command_registry')
        
        # Load internal odometry configuration
        self.internal_odometry = doc.get('internal_odometry', {})
        
        # Load IO/Tool configuration
        self.io_configuration = doc.get('io_configuration', {})

        for j in joints:
            # prefer explicit 'ros_name' for clarity, but fall back to legacy 'name'
            ros_name = j.get('ros_name') or j.get('name')
            if not ros_name:
                continue
            mqtt_name = j.get('mqtt_name') or ros_name
            self.joint_names.append(ros_name)
            self.name_to_mqtt[ros_name] = mqtt_name
            self.mqtt_to_name[mqtt_name] = ros_name
            # allow aliases to be mapped to this joint as well
            for a in j.get('aliases', []) or []:
                self.name_to_mqtt[a] = mqtt_name
                self.mqtt_to_name[a] = ros_name
            # precompile transform fns (forward and reverse). Accept either
            # a singular 'transform' key (commonly used) or 'transforms'.
            transforms_spec = j.get('transform') or j.get('transforms') or {}
            fwd, rev = _make_transform_fns(transforms_spec)
            self.transforms[ros_name] = fwd
            self.reverse_transforms[ros_name] = rev

        self.loaded_at = time.time()

    def should_publish_topic(self, topic_type: str) -> bool:
        """Check if the given topic type (joint, pose) should be published based on capabilities."""
        if self.upstream_mode == 'both':
            return True
        if self.upstream_mode == 'joint' and topic_type == 'joint':
            return True
        if self.upstream_mode == 'pose' and topic_type == 'pose':
            return True
        
        # Fallback to checking the explicit topics list
        return topic_type in self.upstream_topics_list

    # ---------- remap helpers ----------
    def remap_ros_to_mqtt(self, ros_msg: JointState) -> Dict[str, Any]:
        """Convert a ROS JointState into a JSON-serializable dict keyed by
        mqtt names.

        Output layout:
          {"source_type": "edge",
           "positions": {mqtt_name: value, ...},
           "velocities": {...},
           "efforts": {...},
           "ts": <float seconds epoch> }
        """
        positions = {}
        velocities = {}
        efforts = {}

        # build a quick map ros name -> index
        name_to_idx = {n: i for i, n in enumerate(ros_msg.name or [])}

        for ros_name in ros_msg.name or []:
            idx = name_to_idx.get(ros_name)
            mqtt_name = self.name_to_mqtt.get(ros_name)
            if mqtt_name is None:
                # skip joints not in mapping
                continue
            transform = self.transforms.get(ros_name, lambda x: x)
            # positions
            if ros_msg.position and idx is not None and idx < len(ros_msg.position):
                positions[mqtt_name] = transform(ros_msg.position[idx])
            # velocities
            if ros_msg.velocity and idx is not None and idx < len(ros_msg.velocity):
                velocities[mqtt_name] = transform(ros_msg.velocity[idx])
            # efforts
            if ros_msg.effort and idx is not None and idx < len(ros_msg.effort):
                efforts[mqtt_name] = transform(ros_msg.effort[idx])

        out: Dict[str, Any] = {
            "source_type": SOURCE_TYPE_EDGE,  # Upstream traffic from edge device
            "ts": float(ros_msg.header.stamp.sec) + float(ros_msg.header.stamp.nanosec) * 1e-9
        }
        if positions:
            out['positions'] = positions
        if velocities:
            out['velocities'] = velocities
        if efforts:
            out['efforts'] = efforts
        return out

    # DEAD CODE (disabled): debug_print_ros_to_mqtt — defined but never referenced anywhere.
    # def debug_print_ros_to_mqtt(self, ros_msg: JointState, logger: Optional[Any] = None) -> None:
    #     """Print a human-friendly table of how ROS JointState values map to MQTT.

    #     For every joint name present in the ROS message that the mapping knows
    #     about this prints a line showing: ROS name -> MQTT name | ROS value -> MQTT value

    #     If a `logger` is provided it will be used (must support .info()),
    #     otherwise this uses built-in print(). This is intended as a debugging
    #     aid and not for high-throughput production logging.
    #     """
    #     # choose output function
    #     if logger is not None and hasattr(logger, 'info'):
    #         out = logger.info
    #     else:
    #         out = print

    #     name_to_idx = {n: i for i, n in enumerate(ros_msg.name or [])}
    #     out(f"Mapping debug (ros->{'mqtt'}), mapping file={self.path}")
    #     out(f"Timestamp: {float(ros_msg.header.stamp.sec) + float(ros_msg.header.stamp.nanosec) * 1e-9}")
    #     for ros_name in ros_msg.name or []:
    #         idx = name_to_idx.get(ros_name)
    #         mqtt_name = self.name_to_mqtt.get(ros_name)
    #         if mqtt_name is None:
    #             out(f"  SKIP: ROS '{ros_name}' not in mapping")
    #             continue
    #         # get raw ROS values (position only for simplicity)
    #         ros_val = None
    #         try:
    #             if ros_msg.position and idx is not None and idx < len(ros_msg.position):
    #                 ros_val = ros_msg.position[idx]
    #         except Exception:
    #             ros_val = None
    #         # compute mqtt value via forward transform
    #         try:
    #             transform = self.transforms.get(ros_name, lambda x: x)
    #             mqtt_val = transform(ros_val) if ros_val is not None else None
    #         except Exception as e:
    #             mqtt_val = f"<transform error: {e}>"

    #         out(f"  ROS '{ros_name}' -> MQTT '{mqtt_name}': ROS={ros_val!r} -> MQTT={mqtt_val!r}")

    def remap_mqtt_to_ros(self, payload: Any) -> JointState:
        """Convert an MQTT payload (JSON-decoded or string) into a JointState
        message matching this robot's expected names order.

        If a value is missing for a joint, NaN is used in the corresponding
        position slot.
        """
        # accept either a decoded dict/object or a JSON string
        if isinstance(payload, str):
            try:
                data = json.loads(payload)
            except Exception:
                data = {}
        else:
            data = payload or {}

        # determine source positions dict
        positions_src = {}
        if isinstance(data, dict):
            if 'positions' in data and isinstance(data['positions'], dict):
                positions_src = data['positions']
            else:
                # maybe callers sent a flat mapping {mqtt_name: value, ...}
                # treat top-level numeric dict as positions
                # filter only numeric values
                for k, v in data.items():
                    if isinstance(v, (int, float)):
                        positions_src[k] = v

        # construct JointState with mapping.joint_names order
        js = JointState()
        js.header.stamp.sec = int(time.time())
        js.header.stamp.nanosec = int((time.time() % 1.0) * 1e9)
        js.name = list(self.joint_names)
        n = len(js.name)
        js.position = [float('nan')] * n
        js.velocity = []
        js.effort = []

        for i, ros_name in enumerate(self.joint_names):
            mqtt_name = self.name_to_mqtt.get(ros_name)
            if mqtt_name is None:
                continue
            raw_val = positions_src.get(mqtt_name)
            if raw_val is None:
                # try finding by ros name fallback
                raw_val = positions_src.get(ros_name)
            if raw_val is None:
                js.position[i] = float('nan')
            else:
                # apply reverse transform (mqtt->ros)
                rev = self.reverse_transforms.get(ros_name, lambda x: x)
                try:
                    js.position[i] = rev(raw_val)
                except Exception:
                    try:
                        js.position[i] = float(raw_val)
                    except Exception:
                        js.position[i] = float('nan')

        return js


__all__ = ['Mapping']
