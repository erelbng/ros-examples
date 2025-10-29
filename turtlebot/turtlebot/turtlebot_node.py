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
from mujoco import viewer
from nav_msgs.msg import Odometry
from OpenGL.GL import *
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, JointState
from tf2_ros import TransformBroadcaster


class TurtleBotSim(Node):
    def __init__(self):
        super().__init__("turtlebot_sim")

        # === Load MuJoCo model ===
        pkg_share = get_package_share_directory("turtlebot")
        self.model_path = os.path.join(pkg_share, "description", "turtlebot4.xml")

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)

        # === IDs ===
        self.forward_motor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "forward"
        )
        self.turn_motor = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "turn"
        )
        self.left_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_l"
        )
        self.right_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_r"
        )

        # === Parameters ===
        self.L = 0.234  # wheel separation
        self.R = 0.036  # wheel radius

        # === ROS interfaces ===
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.cam_pub = self.create_publisher(Image, "camera", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.imu_pub = self.create_publisher(Imu, "imu", 10)

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

        self.imu_quat_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_quat"
        )
        self.imu_ang_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang"
        )
        self.imu_acc_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_acc"
        )

        # === Camera Implementation ===
        self.renderer = mujoco.Renderer(self.model, 640, 480)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "cam"
        )
        self.bridge = CvBridge()

        # === MuJoCo Viewer ===
        self.viewer = viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        )

    # --- Callback: velocity commands ---
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z
        self.target_vl = v  # (v - omega * self.L / 2.0) / self.R
        self.target_vr = omega  # (v + omega * self.L / 2.0) / self.R

    def get_sensor_data(self, sensor_id):
        """Return the numpy array slice for a given sensor."""
        start = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        return self.data.sensordata[start : start + dim]

    # --- Main simulation loop ---
    def update_loop(self):
        # Apply actuator controls
        self.data.ctrl[self.forward_motor] = self.target_vl
        self.data.ctrl[self.turn_motor] = self.target_vr

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
        js.name = ["wheel_l", "wheel_r"]
        js.position = [q_left, q_right]
        js.velocity = [dq_left, dq_right]
        self.joint_pub.publish(js)

        # Update MuJoCo
        self.viewer.sync()

        # --- Publish Camera ---
        self.renderer.update_scene(self.data, camera=self.cam)
        img_cam = self.renderer.render()
        img_msg = self.bridge.cv2_to_imgmsg(img_cam[..., ::-1], encoding="bgr8")
        self.cam_pub.publish(img_msg)

        # --- Publish IMU ---
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        quat = self.get_sensor_data(self.imu_quat_id)
        gyro = self.get_sensor_data(self.imu_ang_id)
        acc = self.get_sensor_data(self.imu_acc_id)
        # Orientation (quaternion)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]
        self.imu_pub.publish(imu_msg)

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
    node = TurtleBotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down simulation.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
