#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf2_ros
from tf2_ros import TransformException
from std_msgs.msg import Float64MultiArray


class CobotPolicyNode(Node):
    def __init__(self):
        super().__init__("cobot_policy_node")

        # ============================================================
        # CONFIGURATION — MUST MATCH TRAINING EXACTLY
        # (cross-checked against CobotSweepPathRlEnvCfg / ActionsCfg /
        #  ObservationsCfg / CommandsCfg)
        # ============================================================
        self.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        self.num_joints = len(self.joint_names)

        # FIXED: ActionsCfg.arm_action.scale = 0.25, not 0.5.
        self.action_scale = 0.25

        # Default pose offsets from training (JointPositionActionCfg use_default_offset=True).
        # ASSUMPTION: COBOT_CFG's default joint positions are all zero. This wrapper cannot
        # see robot_cfg.py's InitialStateCfg — if COBOT_CFG.init_state.joint_pos is non-zero,
        # this MUST be updated to match, or every action will be offset incorrectly.
        self.default_joint_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

        # TF frame names — MUST match your URDF / robot_state_publisher.
        # FIXED default: rewards/observations key off body_names="ultrasound_tip", not
        # "cobot_ee" (both frames exist in the URDF, but training used ultrasound_tip).
        self.base_frame = self.declare_parameter("base_frame", "ground_link").value
        self.ee_frame = self.declare_parameter("ee_frame", "ultrasound_tip").value

        # Orientation is NOT policy-driven: CommandsCfg.ee_pose fixed
        # roll=pitch=yaw=0 for every single training episode, so the policy has
        # only ever seen this one orientation target. Rather than trusting
        # whatever orientation an upstream publisher (jogger, point-cloud
        # pipeline, etc.) sends on /goal_pose, this node always substitutes the
        # fixed default below when building the observation — see
        # _on_target_pose. Default (1,0,0,0) = identity = "point down", matching
        # training's fixed orientation. Override via this param only if that
        # physical mapping turns out to be wrong.
        self.fixed_target_quat = np.array(
            self.declare_parameter("fixed_target_quat_wxyz", [1.0, 0.0, 0.0, 0.0]).value,
            dtype=np.float32,
        )

        # Staleness guards: without these, a stalled /joint_state_isaac publisher or a
        # stalled TF tree silently freezes joint_pos/ee_pos at their last-known values
        # while the control loop keeps running at 60Hz off a live target — the robot
        # then reacts only to where it's TOLD to go, not where it actually is. Both
        # are checked every control-loop tick, not just once at startup.
        self.max_joint_state_age_sec = self.declare_parameter(
            "max_joint_state_age_sec", 0.2
        ).value
        self.max_tf_age_sec = self.declare_parameter("max_tf_age_sec", 0.2).value
        self.last_joint_state_stamp = None

        # ============================================================
        # LOAD POLICY (ONNX)
        # ============================================================
        policy_path = self.declare_parameter("policy_path", "exported_policy/policy.onnx").value
        # Optional: path to an EXTERNAL normalizer-stats file (json or npz with mean/var or
        # mean/std). Leave empty by default — this export fuses the RSL-RL obs normalizer
        # directly into the ONNX graph, so the graph itself already normalizes internally.
        # Only set this if you later swap in a different ONNX export that does NOT fuse
        # normalization, otherwise obs would be normalized twice.
        norm_stats_path = self.declare_parameter("norm_stats_path", "").value
        providers = self.declare_parameter(
            "onnx_providers", ["CPUExecutionProvider"]
        ).value

        self.session, self.input_name, self.output_name = self._load_onnx_actor(
            policy_path, providers
        )
        self.obs_normalizer = self._load_norm_stats(norm_stats_path)

        # ============================================================
        # ROS 2 INTERFACES
        # ============================================================
        self.sub_joint_state = self.create_subscription(
            JointState, "/joint_state_isaac", self._on_joint_state, 1
        )
        # NOTE: this target is assumed to already be in `base_frame` (ground_link), matching
        # what mdp.generated_commands("ee_pose") produced at training time — UniformPoseCommand
        # generates its command in the asset's ROOT/base frame, not world frame. If whatever
        # publishes /goal_pose is giving you a world-frame target, transform it into base_frame
        # before it reaches this node, or the policy will see an out-of-distribution command.
        self.sub_target = self.create_subscription(
            Pose, "/goal_pose", self._on_target_pose, 1
        )
        self.pub_command = self.create_publisher(
            Float64MultiArray, "/position_controller/commands", 1
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ============================================================
        # STATE
        # ============================================================
        self.joint_pos = np.zeros(self.num_joints, dtype=np.float32)
        self.joint_vel = np.zeros(self.num_joints, dtype=np.float32)
        self.last_action = np.zeros(self.num_joints, dtype=np.float32)
        self.target_pos = np.array([0.25, 0.0, 0.3], dtype=np.float32)
        self.target_quat = self.fixed_target_quat.copy()  # w, x, y, z
        self.has_target = False
        self.has_joint_state = False

        # Control loop at 60 Hz: sim.dt (1/120) * decimation (2) = 1/60. Confirmed to match
        # CobotSweepPathRlEnvCfg.__post_init__.
        self.timer = self.create_timer(1.0 / 60.0, self._control_loop)

        self.get_logger().info(
            f"Node ready. Waiting for TF: {self.base_frame} -> {self.ee_frame} "
            f"and /joint_state_isaac with joints: {self.joint_names}"
        )

    # ------------------------------------------------------------------
    # POLICY LOADING (ONNX)
    # ------------------------------------------------------------------
    def _load_onnx_actor(self, path, providers):
        if not os.path.exists(path):
            raise FileNotFoundError(f"ONNX policy not found: {path}")

        available = ort.get_available_providers()
        chosen = [p for p in providers if p in available] or ["CPUExecutionProvider"]

        session = ort.InferenceSession(path, providers=chosen)

        inputs = session.get_inputs()
        outputs = session.get_outputs()
        if len(inputs) != 1 or len(outputs) < 1:
            self.get_logger().warn(
                f"Unexpected ONNX I/O signature — inputs: {[i.name for i in inputs]}, "
                f"outputs: {[o.name for o in outputs]}. Using the first of each."
            )

        input_name = inputs[0].name
        output_name = outputs[0].name

        self.get_logger().info(f"Loaded ONNX policy from {path}")
        self.get_logger().info(f"  input:  {input_name} shape={inputs[0].shape}")
        self.get_logger().info(f"  output: {output_name} shape={outputs[0].shape}")
        self.get_logger().info(f"  providers: {chosen}")

        expected_obs_dim = 28  # see _compute_observation for the breakdown
        in_shape = inputs[0].shape
        if len(in_shape) == 2 and isinstance(in_shape[1], int) and in_shape[1] != expected_obs_dim:
            self.get_logger().warn(
                f"ONNX input dim ({in_shape[1]}) != expected obs dim ({expected_obs_dim}) "
                f"computed from ObservationsCfg (joint_pos_rel[6] + joint_vel_rel[6] + "
                f"target_pose[7] + ee_pos_err[3] + last_action[6]). Double check the export "
                f"matches training, or that ee_pos_error's true dimensionality matches the "
                f"assumption made here."
            )

        return session, input_name, output_name

    def _load_norm_stats(self, path):
        """Load an optional EXTERNAL obs-normalizer (mean/var or mean/std).

        Not needed for this policy_path — the obs normalizer is fused into the ONNX
        graph itself, so the graph already normalizes internally and this returns None
        (a pass-through) by default. Only populate norm_stats_path if you swap in a
        different ONNX export that does NOT fuse normalization.
        """
        if not path:
            self.get_logger().info(
                "No norm_stats_path set — obs normalizer is fused into the ONNX graph, "
                "so raw observations are passed through unchanged before inference."
            )
            return None

        if not os.path.exists(path):
            raise FileNotFoundError(f"norm_stats_path not found: {path}")

        if path.endswith(".npz"):
            data = np.load(path)
            mean = data["mean"].astype(np.float32)
            std = data["std"].astype(np.float32) if "std" in data else np.sqrt(
                data["var"].astype(np.float32) + 1e-8
            )
        else:
            with open(path, "r") as f:
                data = json.load(f)
            mean = np.array(data["mean"], dtype=np.float32)
            std = (
                np.array(data["std"], dtype=np.float32)
                if "std" in data
                else np.sqrt(np.array(data["var"], dtype=np.float32) + 1e-8)
            )

        self.get_logger().info(f"Loaded external obs normalizer from {path}")
        return {"mean": mean, "std": std}

    def _normalize_obs(self, obs):
        # No-op in the default configuration: the ONNX graph already normalizes
        # internally, so raw observations pass straight through to session.run().
        if self.obs_normalizer is None:
            return obs
        mean = self.obs_normalizer["mean"]
        std = self.obs_normalizer["std"]
        return (obs - mean) / (std + 1e-8)

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_pos[i] = msg.position[idx]
                if msg.velocity and len(msg.velocity) > idx:
                    self.joint_vel[i] = msg.velocity[idx]
        self.has_joint_state = True
        # Record when THIS node received the message, not the message's own header
        # stamp — some Isaac/hardware bridges leave header.stamp at 0, which would
        # make every staleness check below trivially fail (always "infinitely old").
        self.last_joint_state_stamp = self.get_clock().now()

    def _on_target_pose(self, msg: Pose):
        # Position: ASSUMED already in base_frame — see subscription comment above.
        self.target_pos = np.array(
            [msg.position.x, msg.position.y, msg.position.z], dtype=np.float32
        )
        # Orientation: intentionally IGNORED from the incoming message. Always use
        # the fixed default (point-down) set at startup — see comment near
        # fixed_target_quat in __init__ for why.
        self.target_quat = self.fixed_target_quat
        self.has_target = True

    # ------------------------------------------------------------------
    # TF2 FORWARD KINEMATICS
    # ------------------------------------------------------------------
    def _get_ee_pos_from_tf(self):
        """Only position is needed now — ee_pos_error uses position only per
        custom_mdp.ee_pos_error (see caveat in _compute_observation)."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
            )
            tf_age_sec = (self.get_clock().now() - rclpy.time.Time.from_msg(
                trans.header.stamp
            )).nanoseconds / 1e9
            if tf_age_sec > self.max_tf_age_sec:
                self.get_logger().warn(
                    f"TF {self.base_frame}->{self.ee_frame} is stale "
                    f"({tf_age_sec:.3f}s old, limit={self.max_tf_age_sec}s). "
                    f"ee_pos_err will be computed from an outdated position.",
                    throttle_duration_sec=1.0,
                )
            pos = np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ],
                dtype=np.float32,
            )
            return pos
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup {self.base_frame}->{self.ee_frame} failed: {e}",
                throttle_duration_sec=2.0,
            )
            return None

    # ------------------------------------------------------------------
    # OBSERVATION BUILDER
    # ------------------------------------------------------------------
    def _compute_observation(self):
        # 1. Joint positions relative to default — mdp.joint_pos_rel
        joint_pos_rel = self.joint_pos - self.default_joint_pos

        # 2. Joint velocities — mdp.joint_vel_rel
        joint_vel_rel = self.joint_vel

        # 3. Commanded target pose — mdp.generated_commands("ee_pose")
        #    UniformPoseCommandCfg output is (pos[3], quat_wxyz[4]) in the robot's base frame.
        target_pose = np.concatenate([self.target_pos, self.target_quat]).astype(np.float32)

        # 4. End-effector position error — custom_mdp.ee_pos_error
        #    *** ASSUMPTION, NOT VERIFIED ***: implemented here as (target - current ee pos),
        #    a 3-vector, matching the SceneEntityCfg(body_names="ultrasound_tip") used in both
        #    the observation and the position_command_error reward. If custom_mdp.py computes
        #    this differently (different frame, normalization, includes orientation, etc.),
        #    update this block to match exactly — paste custom_mdp.py to verify.
        ee_pos = self._get_ee_pos_from_tf()
        if ee_pos is None:
            self.get_logger().warn("Missing TF, feeding zeros to policy", throttle_duration_sec=1.0)
            ee_pos = np.zeros(3, dtype=np.float32)
        ee_pos_err = (self.target_pos - ee_pos).astype(np.float32)

        # 5. Last action — custom_mdp.last_action_obs (raw, unscaled action)
        last_action = self.last_action

        # Order MUST match ObservationsCfg.PolicyCfg exactly:
        # joint_pos_rel(6) + joint_vel_rel(6) + target_pose(7) + ee_pos_err(3) + last_action(6)
        # = 28 dims total
        obs = np.concatenate(
            [
                joint_pos_rel,   # [0:6]
                joint_vel_rel,   # [6:12]
                target_pose,     # [12:19]
                ee_pos_err,      # [19:22]
                last_action,     # [22:28]
            ]
        ).astype(np.float32)

        return obs.reshape(1, -1)

    # ------------------------------------------------------------------
    # CONTROL LOOP
    # ------------------------------------------------------------------
    def _control_loop(self):
        if not self.has_joint_state:
            self.get_logger().warn("No /joint_state_isaac received yet", throttle_duration_sec=5.0)
            return
        if not self.has_target:
            return

        # Staleness gate: has_joint_state only tells us a message arrived AT SOME POINT.
        # If the publisher stalls afterward, joint_pos/joint_vel would otherwise stay
        # frozen while this loop keeps commanding actions off a live target — i.e. the
        # robot moves toward the goal without actually checking its current position.
        joint_state_age_sec = (
            self.get_clock().now() - self.last_joint_state_stamp
        ).nanoseconds / 1e9
        if joint_state_age_sec > self.max_joint_state_age_sec:
            self.get_logger().warn(
                f"/joint_state_isaac is stale ({joint_state_age_sec:.3f}s old, "
                f"limit={self.max_joint_state_age_sec}s). Holding last command rather "
                f"than acting on outdated feedback.",
                throttle_duration_sec=1.0,
            )
            return

        obs = self._compute_observation()
        obs = self._normalize_obs(obs)

        outputs = self.session.run([self.output_name], {self.input_name: obs})
        action = np.asarray(outputs[0]).flatten().astype(np.float32)

        # Scale to joint deltas (matches JointPositionActionCfg.scale = 0.25)
        scaled_action = action * self.action_scale

        # Add default offset
        target_joint_pos = self.default_joint_pos + scaled_action

        msg = Float64MultiArray()
        msg.data = target_joint_pos.tolist()
        self.pub_command.publish(msg)

        # Store unscaled action for next observation (matches custom_mdp.last_action_obs)
        self.last_action = action


def main(args=None):
    rclpy.init(args=args)
    node = CobotPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
