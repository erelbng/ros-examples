#!/usr/bin/env python3
import math
import os

import cv2
import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from mujoco.glfw import glfw
from nav_msgs.msg import Odometry
from OpenGL.GL import *
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, JointState
from tf2_ros import TransformBroadcaster


class PincherXSim(Node):
    def __init__(self):
        super().__init__("pincherx_sim")

        # === Load MuJoCo model ===
        pkg_share = get_package_share_directory("pincherx")
        self.model_path = os.path.join(pkg_share, "description", "pincherx100.xml")

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        # === ROS interfaces ===
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.cam_pub = self.create_publisher(Image, "camera", 10)
        self.cmd_sub = self.create_subscription(
            JointState, "joint_commands", self.joint_command_callback, 10
        )

        self.actuator_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            for i in range(self.model.nu)
        ]
        self.target_ctrl = np.zeros(self.model.nu)

        self.timer = self.create_timer(0.02, self.update_loop)

        self.sim_dt = self.model.opt.timestep
        self.real_dt = 0.02  # ROS timer period

        self.steps_per_update = int(self.real_dt / self.sim_dt)

        # === Camera Implementation ===
        self.renderer = mujoco.Renderer(self.model, 640, 480)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam"
        )
        self.bridge = CvBridge()

        # === MuJoCo Viewer ===
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        )

    # --- Callback: velocity commands ---
    def joint_command_callback(self, msg: Twist):
        """Receive joint target commands and set controls accordingly."""
        for i, name in enumerate(msg.name):
            if name in self.actuator_names:
                act_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
                self.data.ctrl[act_id] = (
                    msg.position[i] if len(msg.position) > i else 0.0
                )
            else:
                self.get_logger().warn(f"Unknown actuator '{name}' in command")

    # --- Main simulation loop ---
    def update_loop(self):
        # Step simulation
        for _ in range(self.steps_per_update):
            mujoco.mj_step(self.model, self.data)

        # --- Publish Joint States ---
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(self.model.njnt)
        ]
        qpos = self.data.qpos[: self.model.nq]
        qvel = self.data.qvel[: self.model.nv]

        joint_msg.name = joint_names
        joint_msg.position = qpos.tolist()
        joint_msg.velocity = qvel.tolist()

        self.joint_pub.publish(joint_msg)

        # Update MuJoCo
        self.viewer.sync()

        # --- Publish Camera ---
        self.renderer.update_scene(self.data, camera=self.cam)
        img_cam = self.renderer.render()
        img_msg = self.bridge.cv2_to_imgmsg(img_cam[..., ::-1], encoding="bgr8")
        self.cam_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PincherXSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down simulation.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
