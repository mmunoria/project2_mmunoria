import math
from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class EulerToJointNode(Node):
    """
    Subscribes to:
      - /euler_cmd (Vector3: x=roll, y=pitch, z=yaw)
      - /quat_cmd (Quaternion: x, y, z, w)
    Publishes /joint_states for yaw_joint, pitch_joint, roll_joint
    """

    def __init__(self) -> None:
        super().__init__("euler_to_joint_node")

        self.declare_parameter("publish_rate_hz", 50.0)

        # Match the joint names in your URDF exactly
        self.joint_names: List[str] = ["yaw_joint", "pitch_joint", "roll_joint"]
        self.q_yaw = 0.0
        self.q_pitch = 0.0
        self.q_roll = 0.0

        # Subscriptions
        self.euler_sub = self.create_subscription(Vector3, "/euler_cmd", self._on_euler_cmd, 10)
        self.quat_sub = self.create_subscription(Quaternion, "/quat_cmd", self._on_quat_cmd, 10)
        
        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1.0, rate)
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info("Gimbal Node Ready. Use /euler_cmd or /quat_cmd")

    def _on_euler_cmd(self, msg: Vector3) -> None:
        """Standard Euler input: demonstrates gimbal lock at pitch = +/- 90deg."""
        self.q_roll = float(msg.x)
        # Clamping pitch is common to prevent 'flipping' in many Euler controllers
        self.q_pitch = clamp(float(msg.y), -math.pi / 2.0, math.pi / 2.0)
        self.q_yaw = float(msg.z)

    def _on_quat_cmd(self, msg: Quaternion) -> None:
        """Quaternion input: bypasses the mathematical singularity of gimbal lock."""
        # Convert Quat to Euler for the joint state motor positions
        # Note: Scipy uses [x, y, z, w]
        r = R.from_quat([msg.x, msg.y, msg.z, msg.w])
        euler = r.as_euler('zyx', degrees=False) # Yaw, Pitch, Roll
        
        self.q_yaw = euler[0]
        self.q_pitch = euler[1]
        self.q_roll = euler[2]

    def _publish(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.q_yaw, self.q_pitch, self.q_roll]
        self.pub.publish(js)

def main() -> None:
    rclpy.init()
    node = EulerToJointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()