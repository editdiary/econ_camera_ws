#!/usr/bin/env python3
"""pose_logger: /aft_mapped_to_init (nav_msgs/Odometry) -> TUM 궤적 파일.

사용법: python3 pose_logger.py <out.tum>
point_lio와 함께 기동; SIGINT로 종료(종료 시 flush).
"""
import os
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tum import odom_to_tum_line


class PoseLogger(Node):
    def __init__(self, out_path):
        super().__init__("pose_logger")
        self._f = open(out_path, "w")
        self._n = 0
        self.create_subscription(Odometry, "/aft_mapped_to_init", self._cb, 200)

    def _cb(self, msg):
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._f.write(odom_to_tum_line(t, p.x, p.y, p.z, q.x, q.y, q.z, q.w) + "\n")
        self._n += 1

    def destroy_node(self):
        try:
            self._f.flush()
            self._f.close()
        finally:
            super().destroy_node()


def main():
    if len(sys.argv) < 2:
        print("usage: pose_logger.py <out.tum>", file=sys.stderr)
        sys.exit(1)
    out_path = sys.argv[1]
    rclpy.init()
    node = PoseLogger(out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"wrote {node._n} poses to {out_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
