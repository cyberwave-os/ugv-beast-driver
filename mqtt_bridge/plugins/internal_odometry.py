import math
from sensor_msgs.msg import JointState

class InternalOdometry:
    """
    Plugin to calculate internal odometry (dead reckoning) for robots
    that do not provide a native /odom topic.
    """
    def __init__(self, config):
        self.config = config
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.last_left_pos = None
        self.last_right_pos = None
        
        self.track_width = config.get('track_width', 0.231)
        self.wheel_radius = config.get('wheel_radius', 0.051)
        self.left_joints = config.get('left_wheel_joints', [])
        self.right_joints = config.get('right_wheel_joints', [])

    def update(self, msg: JointState):
        """Calculate and update internal odometry based on wheel joint positions."""
        # Build name->position map
        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        
        # helper to get average position of a set of joints
        def get_avg_pos(names):
            vals = [name_to_pos.get(n) for n in names if n in name_to_pos]
            if not vals:
                # Fallback to fuzzy search if explicit joints not found
                for name, pos in name_to_pos.items():
                    n_lower = name.lower()
                    if 'left' in n_lower and ('wheel' in n_lower or 'joint' in n_lower):
                        if names == self.left_joints: vals.append(pos)
                    elif 'right' in n_lower and ('wheel' in n_lower or 'joint' in n_lower):
                        if names == self.right_joints: vals.append(pos)
            return sum(vals) / len(vals) if vals else None

        left_pos = get_avg_pos(self.left_joints)
        right_pos = get_avg_pos(self.right_joints)
        
        if left_pos is not None and right_pos is not None:
            if self.last_left_pos is not None and self.last_right_pos is not None:
                d_left = (left_pos - self.last_left_pos) * self.wheel_radius
                d_right = (right_pos - self.last_right_pos) * self.wheel_radius
                
                d_dist = (d_left + d_right) / 2.0
                d_theta = (d_right - d_left) / self.track_width
                
                # geometry-core-exempt: midpoint integration for a differential
                # drive pose update, not a quaternion half-angle construction.
                self.pose_x += d_dist * math.cos(self.pose_theta + d_theta/2.0)
                self.pose_y += d_dist * math.sin(self.pose_theta + d_theta/2.0)
                self.pose_theta += d_theta
                
                # Normalize theta to [-pi, pi]
                self.pose_theta = (self.pose_theta + math.pi) % (2 * math.pi) - math.pi
                
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos

    def get_pose(self):
        """Return the current calculated pose."""
        return {
            "x": self.pose_x,
            "y": self.pose_y,
            "theta": self.pose_theta
        }

    def reset(self):
        """Reset the internal pose to zero."""
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.last_left_pos = None
        self.last_right_pos = None



