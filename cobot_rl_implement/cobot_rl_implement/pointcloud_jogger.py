#!/usr/bin/env python3

import os
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

import tf2_ros
from tf2_ros import TransformException

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

# Optional scipy import for Euler (deg) -> Quaternion (x, y, z, w) conversion
try:
    from scipy.spatial.transform import Rotation as R
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class JsonJoggerNode(Node):

    def __init__(self, file_path):
        super().__init__("json_jogger_node")

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.base_frame = self.declare_parameter(
            "base_frame",
            "ground_link",
        ).value

        self.ee_frame = self.declare_parameter(
            "ee_frame",
            "ultrasound_tip",
        ).value

        self.pos_tolerance_m = (
            self.declare_parameter(
                "pos_tolerance_mm",
                8.0,
            ).value / 1000.0
        )

        self.verify_timeout_sec = (
            self.declare_parameter(
                "verify_timeout_sec",
                5.0,
            ).value
        )

        verify_rate = self.declare_parameter(
            "verify_poll_rate_hz",
            20.0,
        ).value

        self.verify_poll_period_sec = 1.0 / verify_rate

        self.abort_on_timeout = (
            self.declare_parameter(
                "abort_on_timeout",
                False,
            ).value
        )

        # ============================================================
        # ROS
        # ============================================================

        self.publisher = self.create_publisher(
            Pose,
            "/goal_pose",
            10,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        # ============================================================
        # DATA
        # ============================================================

        self.data_points = self._load_data(file_path)

        self.current_idx = 0
        self.current_target = None
        self.verify_start_time = None

        self.timer = None
        self.verify_timer = None

        if not self.data_points:
            self.get_logger().error("No valid JSON trajectory points.")
            return

        self.get_logger().info(
            f"Loaded {len(self.data_points)} trajectory points."
        )

        # Give ROS / TF time to start.
        self.timer = self.create_timer(
            1.0,
            self._publish_next,
        )

    # ================================================================
    # JSON
    # ================================================================

    def _load_data(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return []

        try:
            with open(file_path, "r") as f:
                data = json.load(f)

            if not isinstance(data, list):
                self.get_logger().error("Expected JSON list.")
                return []

            return data

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
            return []

    # ================================================================
    # TF
    # ================================================================

    def _get_ee_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
            )

            return np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ]
            )

        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed: {e}",
                throttle_duration_sec=1.0,
            )
            return None

    # ================================================================
    # PUBLISH NEXT POINT
    # ================================================================

    def _publish_next(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

        if self.verify_timer is not None:
            self.verify_timer.cancel()
            self.verify_timer = None

        if self.current_idx >= len(self.data_points):
            self.get_logger().info("JSON trajectory complete.")
            rclpy.shutdown()
            return

        sample = self.data_points[self.current_idx]
        msg = Pose()

        # Parse Position (Handles 'xyz_mm' array or fallback 'x', 'y', 'z')
        if "xyz_mm" in sample and isinstance(sample["xyz_mm"], list):
            xyz = sample["xyz_mm"]
            msg.position.x = float(xyz[0]) / 1000.0
            msg.position.y = float(xyz[1]) / 1000.0
            msg.position.z = float(xyz[2]) / 1000.0
        else:
            msg.position.x = float(sample.get("x", 0.0)) / 1000.0
            msg.position.y = float(sample.get("y", 0.0)) / 1000.0
            msg.position.z = float(sample.get("z", 0.0)) / 1000.0

        # Parse Orientation ('euler_deg' -> Quaternion if scipy available)
        if "euler_deg" in sample and HAS_SCIPY:
            euler_deg = sample["euler_deg"]
            quat = R.from_euler("xyz", euler_deg, degrees=True).as_quat()
            msg.orientation.x = float(quat[0])
            msg.orientation.y = float(quat[1])
            msg.orientation.z = float(quat[2])
            msg.orientation.w = float(quat[3])
        else:
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

        self.current_target = np.array(
            [
                msg.position.x,
                msg.position.y,
                msg.position.z,
            ]
        )

        self.publisher.publish(msg)

        point_num = sample.get("i", self.current_idx)
        self.get_logger().info(
            f"Point i={point_num} ({self.current_idx + 1}/{len(self.data_points)}) "
            f"commanded | X={msg.position.x:.3f} Y={msg.position.y:.3f} Z={msg.position.z:.3f}"
        )

        self.current_idx += 1
        self.verify_start_time = self.get_clock().now()

        self.verify_timer = self.create_timer(
            self.verify_poll_period_sec,
            self._check_arrival,
        )

    # ================================================================
    # VERIFY ARRIVAL
    # ================================================================

    def _check_arrival(self):
        ee_pos = self._get_ee_position()

        elapsed = (
            self.get_clock().now() - self.verify_start_time
        ).nanoseconds / 1e9

        error_m = None

        if ee_pos is not None and self.current_target is not None:
            error_m = float(np.linalg.norm(ee_pos - self.current_target))

            if error_m <= self.pos_tolerance_m:
                self.get_logger().info(
                    f"Reached point {self.current_idx}/{len(self.data_points)} | "
                    f"error={error_m * 1000:.2f} mm | time={elapsed:.2f} s"
                )
                self._publish_next()
                return

        if elapsed >= self.verify_timeout_sec:
            error_string = (
                "unknown (no TF)"
                if error_m is None
                else f"{error_m * 1000:.2f} mm"
            )

            if self.abort_on_timeout:
                self.get_logger().error(
                    f"Timed out at point {self.current_idx}/{len(self.data_points)} | "
                    f"error={error_string}"
                )
                if self.verify_timer:
                    self.verify_timer.cancel()
                    self.verify_timer = None
                rclpy.shutdown()
            else:
                self.get_logger().warn(
                    f"Timed out at point {self.current_idx}/{len(self.data_points)} | "
                    f"error={error_string} | continuing"
                )
                self._publish_next()


def main(args=None):
    rclpy.init(args=args)

    try:
        pkg_share = get_package_share_directory("cobot_rl_implement")
        json_path = os.path.join(
            pkg_share,
            "trajectory_points",
            "output.json",
        )
    except PackageNotFoundError:
        print("Package 'cobot_rl_implement' not found.")
        rclpy.shutdown()
        return

    node = JsonJoggerNode(json_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
