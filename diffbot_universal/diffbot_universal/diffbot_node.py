#!/usr/bin/env python3
import math
import os

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class DiffBotSim(Node):
    def __init__(self):
        super().__init__("diffbot_sim")

        # === Load MuJoCo model ===
        pkg_share = get_package_share_directory("diffbot_universal")
        self.model_path = os.path.join(pkg_share, "description", "robot.xml")

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        # === IDs ===
        self.left_motor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_motor"
        )
        self.right_motor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_motor"
        )
        self.left_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint"
        )
        self.right_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint"
        )

        # === Parameters ===
        self.L = 0.15  # wheel separation
        self.R = 0.1  # wheel radius

        # === ROS interfaces ===
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update_loop)

        self.sim_dt = self.model.opt.timestep
        self.real_dt = 0.02  # ROS timer period

        self.steps_per_update = int(self.real_dt / self.sim_dt)

        # === State ===
        self.target_vl = 0.0
        self.target_vr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        )

        self.get_logger().info("✅ DiffBot MuJoCo simulation started.")

    # --- Callback: velocity commands ---
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z
        self.target_vl = (v - omega * self.L / 2.0) / self.R
        self.target_vr = (v + omega * self.L / 2.0) / self.R

    # --- Main simulation loop ---
    def update_loop(self):
        # Apply actuator controls
        self.data.ctrl[self.left_motor] = self.target_vl
        self.data.ctrl[self.right_motor] = self.target_vr

        # Step simulation
        for _ in range(self.steps_per_update):
            mujoco.mj_step(self.model, self.data)

        # Get wheel angles & velocities
        q_left = self.data.qpos[self.left_joint]
        q_right = self.data.qpos[self.right_joint]
        dq_left = self.data.qvel[self.left_joint]
        dq_right = self.data.qvel[self.right_joint]

        # Compute odometry
        v_left = dq_left * self.R
        v_right = dq_right * self.R
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.L

        dt = 0.02
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # --- Publish Joint States ---
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [q_left, q_right]
        js.velocity = [dq_left, dq_right]
        self.joint_pub.publish(js)

        # Update MuJoCo
        self.viewer.sync()

        # --- Publish Odom ---
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # --- Publish TF ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffBotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down simulation.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
