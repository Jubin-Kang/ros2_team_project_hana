#!/usr/bin/env python3
# amcl_follow_bubble_publisher.py
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AmclFollowBubblePublisher(Node):
    """
    /amcl_pose를 구독하여 그 주변(또는 진행방향 ahead 오프셋)에
    반경 radius의 채워진 원형(PointCloud2)을 퍼블리시.
    - 타이머: Steady clock (시뮬/실기 공통 안정)
    - 메시지 stamp: ROS clock(/clock 또는 system time)
    - /amcl_pose 미수신시 5초에 한 번만 경고(쓰로틀)
    """

    def __init__(self):
        super().__init__('amcl_follow_bubble_publisher')

        # ---------- Parameters ----------
        self.declare_parameter('radius', 0.05)            # m
        self.declare_parameter('grid_resolution', 0.01)   # m
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('cloud_topic', '/fixed_bubble_cloud')
        self.declare_parameter('publish_rate', 10.0)      # Hz
        self.declare_parameter('z', 0.0)                  # m
        self.declare_parameter('ahead', 0.0)              # m (진행방향 앞쪽)
        self.declare_parameter('alpha', 1.0)              # 0~1 (1=즉시)

        self.radius = float(self.get_parameter('radius').value)
        self.res    = float(self.get_parameter('grid_resolution').value)
        self.frame  = str(self.get_parameter('frame_id').value)
        self.topic  = str(self.get_parameter('cloud_topic').value)
        self.hz     = float(self.get_parameter('publish_rate').value)
        self.z      = float(self.get_parameter('z').value)
        self.ahead  = float(self.get_parameter('ahead').value)
        self.alpha  = max(0.0, min(1.0, float(self.get_parameter('alpha').value)))

        # ---------- QoS ----------
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---------- Publisher & Subscriber ----------
        self.pub = self.create_publisher(PointCloud2, self.topic, pub_qos)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/DP_08/amcl_pose', self._on_amcl_pose, sub_qos
        )

        # ---------- Precompute disc offsets ----------
        self.offsets = self._make_disc_offsets()

        # ---------- State ----------
        self.has_pose = False
        self.cx = 0.0
        self.cy = 0.0

        # Log throttle state
        self._last_log_time: Time | None = None
        self._log_throttle_sec = 5.0

        # ---------- Timer (steady clock) ----------
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        period = 1.0 / max(1e-3, self.hz)
        self.timer = self.create_timer(period, self._on_timer, clock=self.steady_clock)

        self.get_logger().info(
            f"AMCL-Follow Bubble: r={self.radius}m, res={self.res}m, frame={self.frame}, "
            f"topic={self.topic}, rate={self.hz}Hz, ahead={self.ahead}m, alpha={self.alpha}"
        )

    # ---------- Helpers ----------
    def _make_disc_offsets(self) -> List[Tuple[float, float, float]]:
        r, res = self.radius, self.res
        pts: List[Tuple[float, float, float]] = []
        x = -r
        while x <= r + 1e-9:
            y = -r
            while y <= r + 1e-9:
                if x * x + y * y <= r * r:
                    pts.append((x, y, 0.0))
                y += res
            x += res
        return pts

    def _throttled_warn(self, msg: str):
        """5초에 한 번만 warn 로그."""
        now = self.get_clock().now()
        if self._last_log_time is None:
            self.get_logger().warn(msg)
            self._last_log_time = now
            return
        dt = (now - self._last_log_time).nanoseconds / 1e9
        if dt >= self._log_throttle_sec:
            self.get_logger().warn(msg)
            self._last_log_time = now

    # ---------- Callbacks ----------
    def _on_amcl_pose(self, p: PoseWithCovarianceStamped):
        px = p.pose.pose.position.x
        py = p.pose.pose.position.y
        q = p.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # 진행방향 ahead 오프셋 적용
        target_x = px + self.ahead * math.cos(yaw)
        target_y = py + self.ahead * math.sin(yaw)

        if not self.has_pose:
            self.cx, self.cy = target_x, target_y
            self.has_pose = True
        else:
            self.cx = self.alpha * target_x + (1.0 - self.alpha) * self.cx
            self.cy = self.alpha * target_y + (1.0 - self.alpha) * self.cy

    def _on_timer(self):
        if not self.has_pose:
            self._throttled_warn("Waiting for /amcl_pose...")
            return

        # 메시지 stamp는 ROS clock(시뮬/실기 자동)
        ros_now = self.get_clock().now()

        header = Header()
        header.stamp = ros_now.to_msg()
        header.frame_id = self.frame

        pts = [(self.cx + ox, self.cy + oy, self.z) for (ox, oy, _z) in self.offsets]
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub.publish(cloud)


def main():
    rclpy.init()
    node = AmclFollowBubblePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
