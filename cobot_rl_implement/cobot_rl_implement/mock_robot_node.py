#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np


class MockRobotNode(Node):
    def __init__(self):
        super().__init__("mock_robot_node")

        self.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        self.num_joints = len(self.joint_names)

        # Initial joint positions (match your default pose)
        self.joint_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.joint_vel = np.zeros(self.num_joints)

        # Subscribe to RL policy commands
        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            "/joint_group_position_controller/commands",
            self._on_command,
            1,
        )

        # Publish joint states
        self.pub_joint_state = self.create_publisher(JointState, "/joint_states", 10)

        # Timer at 60 Hz to simulate physics loop
        self.dt = 1.0 / 60.0
        self.timer = self.create_timer(self.dt, self._tick)

        self.target_pos = self.joint_pos.copy()
        self.get_logger().info("Mock robot running. Listening to /joint_group_position_controller/commands")

    def _on_command(self, msg):
        if len(msg.data) != self.num_joints:
            self.get_logger().warn(f"Expected {self.num_joints} commands, got {len(msg.data)}")
            return
        self.target_pos = np.array(msg.data)

    def _tick(self):
        # Simple first-order lag: move current position toward target
        # Time constant ~0.1s
        alpha = 0.3
        self.joint_vel = (self.target_pos - self.joint_pos) / self.dt * alpha
        self.joint_pos += self.joint_vel * self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_pos.tolist()
        msg.velocity = self.joint_vel.tolist()
        self.pub_joint_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
