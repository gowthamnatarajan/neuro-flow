#!/usr/bin/env python3
"""Collects (observation, action) pairs from the real JetAuto Pro.

Subscribes to:
  /neuro_flow/target_state               Float32MultiArray  - commanded joint angles (action)
  /neuro_flow/record_cmd                 String             - "start" | "pause" | "save" | "discard"
  <camera_topic>                         Image              - RGB camera
  /controller_manager/joint_states       JointState         - actual joint angles in radians (state)

/controller_manager/joint_states is a standard sensor_msgs/JointState topic that the robot
publishes, with names: joint1-5 + r_joint. Falls back to commanded angles if unavailable.
"""

import time

import cv_bridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String

from .hdf5_writer import HDF5Writer

# Joint order as stored in HDF5: must match teleop's target_state order
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"]
NUM_JOINTS = len(JOINT_NAMES)


class CollectorNode(Node):
    def __init__(self):
        super().__init__("collector_node")

        self.declare_parameter("camera_topic", "/usb_cam/image_raw")
        self.declare_parameter("fps", 25)
        self.declare_parameter("output_dir", "~/projects/neuro-flow/datasets")
        self.declare_parameter("task", "cube_pick")

        camera_topic = self.get_parameter("camera_topic").value
        fps = self.get_parameter("fps").value
        output_dir = self.get_parameter("output_dir").value
        task = self.get_parameter("task").value

        self._writer = HDF5Writer(output_dir, task, fps)
        self._bridge = cv_bridge.CvBridge()
        self._fps = fps
        self._recording = False

        # Episode buffers
        self._images: list = []
        self._states: list = []
        self._actions: list = []
        self._timestamps: list = []

        # Latest values
        self._latest_image = None
        self._latest_state: list | None = None   # actual joint angles (rad)
        self._latest_action: list | None = None  # commanded angles (rad)

        qos_reliable = QoSProfile(depth=10)
        qos_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, camera_topic, self._on_image, qos_sensor)
        self.create_subscription(Float32MultiArray, "/neuro_flow/target_state", self._on_action, qos_reliable)
        self.create_subscription(String, "/neuro_flow/record_cmd", self._on_cmd, qos_reliable)

        # /controller_manager/joint_states is a standard JointState — no custom msgs needed
        self.create_subscription(
            JointState,
            "/controller_manager/joint_states",
            self._on_joint_states,
            qos_sensor,
        )

        self.create_timer(1.0 / fps, self._capture_frame)

        self.get_logger().info(
            f"Collector ready | camera: {camera_topic} | fps: {fps} | output: {output_dir}"
        )

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)

    def _on_action(self, msg: Float32MultiArray):
        self._latest_action = list(msg.data[:NUM_JOINTS])

    def _on_joint_states(self, msg: JointState):
        """Parse /controller_manager/joint_states into ordered float list."""
        name_to_pos = dict(zip(msg.name, msg.position))
        self._latest_state = [
            float(name_to_pos.get(n, 0.0)) for n in JOINT_NAMES
        ]

    def _on_cmd(self, msg: String):
        cmd = msg.data
        if cmd == "start":
            self._recording = True
            self.get_logger().info("Recording started")
        elif cmd == "pause":
            self._recording = False
            self.get_logger().info("Recording paused")
        elif cmd == "save":
            self._recording = False
            self._save_episode()
        elif cmd == "discard":
            self._recording = False
            self._clear_buffers()
            self.get_logger().info("Episode discarded")

    # ── Capture timer ─────────────────────────────────────────────────────────

    def _capture_frame(self):
        if not self._recording:
            return
        if self._latest_image is None or self._latest_action is None:
            return

        # Use actual joint angles if available, else fall back to commanded
        state = self._latest_state if self._latest_state is not None else self._latest_action

        self._images.append(self._latest_image.copy())
        self._states.append(list(state))
        self._actions.append(list(self._latest_action))
        self._timestamps.append(time.time())

    # ── Episode management ────────────────────────────────────────────────────

    def _save_episode(self):
        n = len(self._actions)
        if n < 5:
            self.get_logger().warn(f"Episode too short ({n} frames) — discarding")
            self._clear_buffers()
            return

        path = self._writer.save_episode(
            self._images, self._states, self._actions, self._timestamps
        )
        self.get_logger().info(f"Saved: {path}  ({n} frames)")
        self._clear_buffers()

    def _clear_buffers(self):
        self._images.clear()
        self._states.clear()
        self._actions.clear()
        self._timestamps.clear()


def main():
    rclpy.init()
    node = CollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
