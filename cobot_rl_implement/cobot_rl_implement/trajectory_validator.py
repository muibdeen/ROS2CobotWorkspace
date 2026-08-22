#!/usr/bin/env python3
"""
trajectory_validator.py

Standalone, OFFLINE validation tool. This does NOT publish /goal_pose and has
NOTHING to do with your RL runtime pipeline (CobotPolicyNode). Its only job is
to ask a running MoveIt `move_group` instance, one time, "is this pose
reachable, and does the IK solution self-collide?" for every point in your
trajectory JSON, then write out a filtered file of only the good points.

Requires: a MoveIt config package for cobot32opi with an SRDF (self-collision
matrix) and a move_group node running. You do NOT need real hardware/controllers
for this — launching your MoveIt config's demo.launch.py (fake controllers) is
enough, since we only ever call the /compute_ik service.

Usage:
    # terminal 1
    ros2 launch <your_cobot>_moveit_config demo.launch.py

    # terminal 2
    ros2 run cobot_rl_implement trajectory_validator.py \
        --ros-args -p planning_group:=arm -p ik_link_name:=ultrasound_tip
"""
import os
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


MOVEIT_ERROR_STRINGS = {
    1: "SUCCESS",
    -10: "START_STATE_IN_COLLISION",
    -12: "GOAL_IN_COLLISION",
    -21: "FRAME_TRANSFORM_FAILURE",
    -31: "NO_IK_SOLUTION",
    -15: "INVALID_GROUP_NAME",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
}


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class TrajectoryValidator(Node):
    def __init__(self, input_path):
        super().__init__('trajectory_validator')

        # ---- CONFIGURE FOR YOUR ROBOT ----
        self.declare_parameter('planning_group', 'cobot_arm')            # your MoveIt planning group
        self.declare_parameter('ik_link_name', 'link_6')   # end-effector link (matches CobotPolicyNode's ee_frame)
        self.declare_parameter('base_frame', 'ground_link')        # matches CobotPolicyNode's base_frame
        self.declare_parameter('angles_in_degrees', True)
        self.declare_parameter('ik_timeout_sec', 0.5)

        # Microadjustment margin: a point only counts as valid if the arm can
        # ALSO reach every corner of a cube of this half-width around it
        # (not just the exact commanded point), so small on-the-fly nudges
        # never land on an unreachable / self-colliding pose.
        self.declare_parameter('microadjust_margin_mm', 3.0)
        self.declare_parameter('check_microadjust_corners', True)
        # -----------------------------------

        self.planning_group = self.get_parameter('planning_group').value
        self.ik_link_name = self.get_parameter('ik_link_name').value
        self.base_frame = self.get_parameter('base_frame').value
        self.angles_in_degrees = self.get_parameter('angles_in_degrees').value
        self.ik_timeout_sec = self.get_parameter('ik_timeout_sec').value
        self.microadjust_margin_m = self.get_parameter('microadjust_margin_mm').value / 1000.0
        self.check_microadjust_corners = self.get_parameter('check_microadjust_corners').value

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.get_logger().info("Waiting for /compute_ik service (move_group must be running)...")
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "/compute_ik service not available. Is move_group running for your "
                "cobot32opi MoveIt config? (e.g. `ros2 launch <pkg>_moveit_config demo.launch.py`)"
            )

        self.input_path = input_path
        self.raw_points = self._load_data(input_path)

    def _load_data(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return []
        with open(file_path, 'r') as f:
            data = json.load(f)
        if not isinstance(data, list):
            self.get_logger().error("JSON format unexpected! Expected a list of dictionaries.")
            return []
        return data

    def _sample_to_pose_stamped(self, sample):
        x = float(sample.get('x', 0.0)) / 1000.0
        y = float(sample.get('y', 0.0)) / 1000.0
        z = float(sample.get('z', 0.0)) / 1000.0

        r = float(sample.get('r', 0.0))
        p = float(sample.get('p', 0.0))
        # NOTE: check this key — your position dict already uses 'y'. Make sure
        # your yaw field has a distinct key (e.g. 'yaw') in the source JSON.
        yaw = float(sample.get('yaw', 0.0))

        if self.angles_in_degrees:
            r, p, yaw = math.radians(r), math.radians(p), math.radians(yaw)
        qx, qy, qz, qw = euler_to_quaternion(r, p, yaw)

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _offset_pose_stamped(self, base_ps: PoseStamped, dx, dy, dz):
        """Return a copy of base_ps translated by (dx, dy, dz) meters, same orientation."""
        ps = PoseStamped()
        ps.header.frame_id = base_ps.header.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = base_ps.pose.position.x + dx
        ps.pose.position.y = base_ps.pose.position.y + dy
        ps.pose.position.z = base_ps.pose.position.z + dz
        ps.pose.orientation = base_ps.pose.orientation
        return ps

    def _generate_microadjust_offsets(self, base_ps: PoseStamped):
        """
        Build the set of perturbed poses to also verify around base_ps, covering
        the worst-case extremes of a +/- margin cube (the 8 corners). Orientation
        is held fixed — this is purely a positional reachability/collision margin.
        """
        m = self.microadjust_margin_m
        offsets = []
        if self.check_microadjust_corners:
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        label = f"corner({sx*m*1000:+.1f},{sy*m*1000:+.1f},{sz*m*1000:+.1f})mm"
                        offsets.append((label, self._offset_pose_stamped(base_ps, sx * m, sy * m, sz * m)))
        else:
            # Cheaper axis-aligned-only check (6 probes instead of 8 corners).
            for axis, sign in (('x', 1), ('x', -1), ('y', 1), ('y', -1), ('z', 1), ('z', -1)):
                dx = m * sign if axis == 'x' else 0.0
                dy = m * sign if axis == 'y' else 0.0
                dz = m * sign if axis == 'z' else 0.0
                label = f"{axis}{'+' if sign > 0 else '-'}{m*1000:.1f}mm"
                offsets.append((label, self._offset_pose_stamped(base_ps, dx, dy, dz)))
        return offsets

    def _check_pose(self, pose_stamped: PoseStamped):
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = self.planning_group
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.robot_state = RobotState()
        req.ik_request.avoid_collisions = True   # <-- self-collision check happens here
        req.ik_request.timeout = Duration(seconds=self.ik_timeout_sec).to_msg()
        req.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.ik_timeout_sec + 1.0)

        if not future.done() or future.result() is None:
            return False, "SERVICE_CALL_TIMEOUT", None

        result = future.result()
        err_code = result.error_code.val
        err_str = MOVEIT_ERROR_STRINGS.get(err_code, f"ERROR_CODE_{err_code}")
        joint_solution = None
        if err_code == MoveItErrorCodes.SUCCESS:
            joint_solution = dict(zip(
                result.solution.joint_state.name,
                result.solution.joint_state.position,
            ))
        return err_code == MoveItErrorCodes.SUCCESS, err_str, joint_solution

    def run(self):
        if not self.raw_points:
            self.get_logger().error("No points loaded, nothing to validate.")
            return

        valid_points = []
        report = []
        for idx, sample in enumerate(self.raw_points):
            base_ps = self._sample_to_pose_stamped(sample)

            center_ok, center_reason, joint_solution = self._check_pose(base_ps)

            # Only bother probing the margin cube if the exact point itself
            # is reachable/collision-free — no point burning IK calls on
            # perturbations of an already-rejected pose.
            margin_results = []
            margin_all_ok = True
            if center_ok:
                for label, offset_ps in self._generate_microadjust_offsets(base_ps):
                    ok_i, reason_i, _ = self._check_pose(offset_ps)
                    margin_results.append({"offset": label, "valid": ok_i, "reason": reason_i})
                    if not ok_i:
                        margin_all_ok = False
            else:
                margin_all_ok = False

            point_valid = center_ok and margin_all_ok

            report.append({
                "index": idx,
                "input": sample,
                "center_valid": center_ok,
                "center_reason": center_reason,
                "margin_mm": self.microadjust_margin_m * 1000.0,
                "margin_checks": margin_results,
                "valid": point_valid,
                "ik_solution": joint_solution,
            })

            if point_valid:
                status = "OK (incl. +/-margin)"
            elif center_ok:
                failed = [r["offset"] for r in margin_results if not r["valid"]]
                status = f"REJECT (center OK, margin fails at: {failed})"
            else:
                status = f"REJECT (center: {center_reason})"
            self.get_logger().info(f"[{idx+1}/{len(self.raw_points)}] {status}: {sample}")

            if point_valid:
                valid_points.append(sample)

        base, ext = os.path.splitext(self.input_path)
        report_path = f"{base}_validation_report.json"
        filtered_path = f"{base}_validated{ext}"

        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        with open(filtered_path, 'w') as f:
            json.dump(valid_points, f, indent=2)

        self.get_logger().info(
            f"Done: {len(valid_points)}/{len(self.raw_points)} points passed. "
            f"Full report: {report_path} | Filtered points for playback: {filtered_path}"
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        pkg_share = get_package_share_directory("cobot_rl_implement")
        json_path = os.path.join(pkg_share, "trajectory_points", "output.json")
    except PackageNotFoundError:
        print("Error: Package 'cobot_rl_implement' not found. Ensure your workspace is sourced.")
        rclpy.shutdown()
        return

    node = TrajectoryValidator(json_path)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
