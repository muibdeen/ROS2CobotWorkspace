#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import torch
import numpy as np
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
        self.action_scale = 0.5
        self.device = torch.device("cuda")

        # Default pose offsets from training (JointPositionActionCfg use_default_offset=True)
        self.default_joint_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # TF frame names — MUST match your URDF / robot_state_publisher
        self.base_frame = self.declare_parameter("base_frame", "ground_link").value
        self.ee_frame = self.declare_parameter("ee_frame", "ultrasound_tip").value  # or "ultrasound_tip"


        # ============================================================
        # LOAD POLICY
        # ============================================================
        policy_path = self.declare_parameter("policy_path", "exported_policy/actor.onnx").value
        self.actor = self._load_actor(policy_path)
        self.actor.eval()

        # ============================================================
        # ROS 2 INTERFACES
        # ============================================================
        self.sub_joint_state = self.create_subscription(
            JointState, "/joint_state_isaac", self._on_joint_state, 1
        )
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
        self.joint_pos = np.zeros(self.num_joints)
        self.joint_vel = np.zeros(self.num_joints)
        self.last_action = np.zeros(self.num_joints)
        self.target_pos = np.array([0.25, 0.0, 0.3])
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.has_target = False
        self.has_joint_state = False

        # Control loop at 60 Hz (must match training sim rate)
        self.timer = self.create_timer(1.0 / 60.0, self._control_loop)

        self.get_logger().info(
            f"Node ready. Waiting for TF: {self.base_frame} -> {self.ee_frame} "
            f"and /joint_states with joints: {self.joint_names}"
        )

    # ------------------------------------------------------------------
    # POLICY LOADING
    # ------------------------------------------------------------------
    def _load_actor(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"Policy not found: {path}")

        checkpoint = torch.load(path, map_location=self.device)

        # ------------------------------------------------------------------
        # Debug: inspect all keys
        # ------------------------------------------------------------------
        self.get_logger().info("Checkpoint keys:")
        for k, v in checkpoint.items():
            if isinstance(v, torch.Tensor):
                self.get_logger().info(f"  {k}: {v.shape}")
            else:
                self.get_logger().info(f"  {k}: {type(v).__name__}")

        # ------------------------------------------------------------------
        # Auto-detect MLP architecture from all mlp.*.weight keys
        # ------------------------------------------------------------------
        mlp_weight_keys = sorted([
            k for k in checkpoint.keys()
            if k.startswith("mlp.") and k.endswith(".weight")
        ], key=lambda x: int(x.split(".")[1]))

        if not mlp_weight_keys:
            raise RuntimeError("No 'mlp.*.weight' keys found in checkpoint")

        # Build input/output dimensions list
        dims = []
        for k in mlp_weight_keys:
            w = checkpoint[k]
            if not dims:
                dims.append(w.shape[1])  # input dimension (obs_dim)
            dims.append(w.shape[0])      # output dimension of this layer

        self.get_logger().info(f"Detected MLP layers: {mlp_weight_keys}")
        self.get_logger().info(f"Detected MLP dimensions: {dims}")

        # Build matching Sequential dynamically
        layers = []
        for i in range(len(dims) - 1):
            layers.append(torch.nn.Linear(dims[i], dims[i + 1]))
            # Add ELU between layers, but NOT after the final output layer
            if i < len(dims) - 2:
                layers.append(torch.nn.ELU())

        actor = torch.nn.Sequential(*layers).to(self.device)

        # Load weights (strip 'mlp.' prefix to match Sequential indexing)
        mlp_state = {
            k.replace("mlp.", ""): v
            for k, v in checkpoint.items()
            if k.startswith("mlp.")
        }
        actor.load_state_dict(mlp_state)

        # ------------------------------------------------------------------
        # Load observation normalizer
        # ------------------------------------------------------------------
        self.obs_normalizer = None
        norm_keys = [k for k in checkpoint.keys() if k.startswith("obs_normalizer.")]
        if norm_keys:
            norm_state = {
                k.replace("obs_normalizer.", ""): v
                for k, v in checkpoint.items()
                if k.startswith("obs_normalizer.")
            }
            self.obs_normalizer = {
                "mean": norm_state["_mean"].to(self.device),
                "var": norm_state["_var"].to(self.device),
                "std": norm_state["_std"].to(self.device),
            }
            self.get_logger().info("Loaded observation normalizer")
        else:
            self.get_logger().warn("No observation normalizer found in checkpoint")

        self.get_logger().info(f"Successfully loaded actor from {path}")
        return actor
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

    def _on_target_pose(self, msg: Pose):
        self.target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.target_quat = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])
        self.has_target = True
    def _normalize_obs(self, obs):
        if self.obs_normalizer is None:
            return obs
        mean = self.obs_normalizer["mean"]
        std = self.obs_normalizer["std"]
        return (obs - mean) / (std + 1e-8)
    # ------------------------------------------------------------------
    # TF2 FORWARD KINEMATICS
    # ------------------------------------------------------------------
    def _get_ee_pose_from_tf(self):
        try:
            # lookup_transform(target_frame, source_frame, time)
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
            )
            pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ])
            quat = np.array([
                trans.transform.rotation.w,
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
            ])
            return pos, quat
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup {self.base_frame}->{self.ee_frame} failed: {e}",
                throttle_duration_sec=2.0,
            )
            return None, None

    # ------------------------------------------------------------------
    # MATH UTILS
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_mul(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    @staticmethod
    def _quat_conjugate(q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

# ------------------------------------------------------------------
    # OBSERVATION BUILDER
    # ------------------------------------------------------------------
    def _compute_observation(self):
        # 1. Joint positions relative to default
        joint_pos_rel = self.joint_pos - self.default_joint_pos

        # 2. Joint velocities
        joint_vel_rel = self.joint_vel

        # 3. Current End Effector Pose (from TF)
        ee_pos, ee_quat = self._get_ee_pose_from_tf()
        if ee_pos is None:
            # Prevent crash if TF drops, though it invalidates this step
            self.get_logger().warn("Missing TF, feeding zeros to policy", throttle_duration_sec=1.0)
            ee_pos = np.zeros(3)
            ee_quat = np.array([1.0, 0.0, 0.0, 0.0])
        
        ee_pose = np.concatenate([ee_pos, ee_quat])

        # 4. Target pose command (position + quaternion)
        target_pose = np.concatenate([self.target_pos, self.target_quat])
        
        # 5. Last action
        last_action = self.last_action

        # MUST match the order and dimensions of ObservationsCfg exactly!
        # joint_pos(6) + joint_vel(6) + ee_pose(7) + pose_command(7) + actions(6) = 32 dims
        obs = np.concatenate([
            joint_pos_rel,      # [0:6]
            joint_vel_rel,      # [6:12]
            ee_pose,            # [12:19]
            target_pose,        # [19:26]
            last_action         # [26:32]
        ]).astype(np.float32)

        return torch.from_numpy(obs).unsqueeze(0).to(self.device)

    # ------------------------------------------------------------------
    # CONTROL LOOP
    # ------------------------------------------------------------------
    def _control_loop(self):
        if not self.has_joint_state:
            self.get_logger().warn("No /joint_state_isaac received yet", throttle_duration_sec=5.0)
            return
        if not self.has_target:
            return

        with torch.no_grad():
            obs = self._compute_observation()
            obs = self._normalize_obs(obs)
            action = self.actor(obs).cpu().numpy().flatten()

            # Scale to joint deltas (matches JointPositionActionCfg)
            scaled_action = action * self.action_scale

            # Add default offset
            target_joint_pos = self.default_joint_pos + scaled_action

            # Publish as JointState
            msg = Float64MultiArray()
            msg.data = target_joint_pos.tolist()
 
            self.pub_command.publish(msg)

            # Store unscaled action for next observation
            self.last_action = action


def main(args=None):
    rclpy.init(args=args)
    node = CobotPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
