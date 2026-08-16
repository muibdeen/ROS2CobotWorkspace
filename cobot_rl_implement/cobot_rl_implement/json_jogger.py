#!/usr/bin/env python3
import os
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

class JsonJoggerNode(Node):
    def __init__(self, file_path):
        super().__init__('json_jogger_node')
        
        self.publisher = self.create_publisher(Pose, '/goal_pose', 10)
        
        self.data_points = self._load_data(file_path)
        self.current_idx = 0
        
        if not self.data_points:
            self.get_logger().error("No valid JSON data found in the file!")
            return

        self.get_logger().info(f"Loaded {len(self.data_points)} valid poses. Starting playback...")
        
        # Kick off the playback loop with a fixed rate (10 Hz) since t_ns is removed
        self.timer = self.create_timer(3.0, self._publish_next)

    def _load_data(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return []

        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                
                # Target the 'samples' array based on your file structure
                if "samples" in data and isinstance(data["samples"], list):
                    return data["samples"]
                else:
                    self.get_logger().error("JSON found, but the 'samples' array is missing!")
                    return []
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
            return []

    def _publish_next(self):
        if self.timer is not None:
            self.timer.cancel()

        if self.current_idx >= len(self.data_points):
            self.get_logger().info("Playback complete. Shutting down...")
            rclpy.shutdown()
            return

        # Get current sample
        sample = self.data_points[self.current_idx]
        
        # Safely extract the 3 XYZ components in millimeters
        coords = sample.get('xyz_mm', [0.0, 0.0, 0.0])

        msg = Pose()
        
        # Convert millimeters to meters
        msg.position.x = float(coords[0]) / 1000.0
        msg.position.y = float(coords[1]) / 1000.0
        msg.position.z = float(coords[2]) / 1000.0

        # Keeping a valid quaternion structure without forcing orientation.x/y 
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Point {self.current_idx + 1}/{len(self.data_points)} | "
            f"X: {msg.position.x:.3f}, Y: {msg.position.y:.3f}, Z: {msg.position.z:.3f}"
        )

        # Schedule the next publication
        self.timer = self.create_timer(.1, self._publish_next)
            
        self.current_idx += 1


def main(args=None):
    rclpy.init(args=args)
    
    try:
        pkg_share = get_package_share_directory("cobot_rl_implement")
        json_path = os.path.join(pkg_share, "trajectory_points", "clamp15_taught_path.json")
    except PackageNotFoundError:
        print("Error: Package 'cobot_rl_implement' not found. Ensure your workspace is sourced.")
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

if __name__ == '__main__':
    main()
