---
layout: default
title: ros-examples
description: ROS 2 robot examples with MuJoCo and Gazebo.
image: assets/tb4_sim_poster.jpg
tldr: >-
  ROS 2 packages for a differential-drive robot, a TurtleBot 4 and a PincherX 100,
  simulated in Gazebo and [MuJoCo](https://mujoco.org) and controlled through standard ROS topics.

authors:
  - name: Eric Plaß
    url: https://github.com/erelbng
links:
  - name: Code
    url: https://github.com/erelbng/ros-examples
    icon: github
  - name: Python version
    url: https://github.com/erelbng/mujoco-examples
  - name: BibTeX
    url: "#citation"
logos:
  - name: HTWK Leipzig
    src: assets/logos/htwk.svg
    url: https://www.htwk-leipzig.de
    height: 24
  - name: Fakultät Ingenieurwissenschaften, HTWK Leipzig
    src: assets/logos/htwk-fing.png
    url: https://fing.htwk-leipzig.de
    height: 34

video_rows:
  - title: In simulation
    caption: TurtleBot 4 and PincherX 100 running in MuJoCo
    videos:
      - title: TurtleBot 4
        caption: Driving with cmd_vel, streaming odometry and camera
        src: assets/tb4_sim.mp4
        poster: assets/tb4_sim_poster.jpg
      - title: PincherX 100
        caption: Pick and place through a sequence of joint poses
        src: assets/pincherx_sim.mp4
        poster: assets/pincherx_sim_poster.jpg

footer: >-
  Built with [ROS 2](https://docs.ros.org), [MuJoCo](https://mujoco.org) and [OpenCV](https://opencv.org).
---

## About

The packages range from a classic Gazebo setup to lightweight MuJoCo simulations. The MuJoCo packages do not need Gazebo or ros2_control: each one is a single Python node that steps the physics, renders the camera and publishes the results as ROS topics.

For the same robots without ROS, see [mujoco-examples](https://github.com/erelbng/mujoco-examples). It runs directly in Python, **Windows included**.

It is designed for:
- **Teaching** ROS 2 concepts such as topics, TF and teleoperation on simulated robots
- **Computer vision** on the `/camera` image stream
- **Data science** on IMU, odometry and joint states
- **Comparing simulators**, with the same diffbot in Gazebo and in MuJoCo

## How it works

Each MuJoCo package has a launch file that starts its node with `mjpython`. The node loads the MJCF model, opens the MuJoCo passive viewer and runs a 20 ms ROS timer. On each tick, it applies the latest command, advances the physics and publishes the robot state.

```
ROS 2 tools      teleop_twist_keyboard, ros2 topic, rviz2
   |    ^
   |    |        down: /cmd_vel, /joint_commands
   v    |        up:   /camera, /imu, /joint_states, /odom, /tf
ROS 2 node       *_node.py, 50 Hz timer
   |    ^
   |    |        down: ctrl
   v    |        up:   qpos, qvel, camera pixels
MuJoCo           mj_step, offscreen renderer, passive viewer
```

## Robots

### diffbot

An introductory example based on [articubot_one](https://github.com/joshnewans/articubot_one) that implements a simple differential-drive robot with *ros2_control*, *Ignition Gazebo* and *rviz2*. Works on *Linux/Ubuntu* only. Source: [`diffbot/`](https://github.com/erelbng/ros-examples/tree/main/diffbot)

```bash
ros2 launch diffbot launch_sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/diff_cont/cmd_vel
```

### diffbot_universal

The diffbot, but with its own control implementation and a *MuJoCo* simulator that runs on more than one OS. Source: [`diffbot_universal/`](https://github.com/erelbng/ros-examples/tree/main/diffbot_universal)

| Subscribes | Publishes |
|---|---|
| `/cmd_vel` | `/joint_states`, `/odom`, `/tf` |

### [TurtleBot 4](https://clearpathrobotics.com/turtlebot-4/)

A differential-drive mobile robot in a warehouse-style scene, simulated with *MuJoCo* and *OpenCV*. It publishes a 640×480 camera image, IMU data, odometry and TF. Source: [`turtlebot/`](https://github.com/erelbng/ros-examples/tree/main/turtlebot)

| Subscribes | Publishes |
|---|---|
| `/cmd_vel` | `/camera`, `/imu`, `/joint_states`, `/odom`, `/tf` |

### [PincherX 100](https://www.trossenrobotics.com/pincherx100)

A compact 4-DOF arm with a gripper, mounted on a workbench. Use it for teleoperation and pick-and-place. Source: [`pincherx/`](https://github.com/erelbng/ros-examples/tree/main/pincherx)

| Subscribes | Publishes |
|---|---|
| `/joint_commands` | `/camera`, `/joint_states` |

```bash
ros2 topic pub /joint_commands sensor_msgs/JointState "
header:
  stamp: {sec: 0, nanosec: 0}
name: ['waist', 'shoulder', 'elbow', 'wrist', 'gripper']
position: [0.0, 0.6, -0.8, 0.4, 0.2]
velocity: []
effort: []
" -1
```

## Quick start

1. Clone the repository into a colcon workspace and build it:
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/erelbng/ros-examples.git
   cd ~/ros2_ws
   colcon build
   source install/local_setup.sh
   ```
2. Start a simulation:
   ```bash
   ros2 launch turtlebot launch_sim.launch.py
   ```
3. In a second terminal, drive the robot with the keyboard:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. Watch the topics with `ros2 topic echo /imu`, or open `rviz2` for the camera image.

## Citation

If you use **ros-examples** in your work, please cite it as:

```bibtex
@misc{ros2025examples,
  author = {Eric Elbing},
  title  = {ros-examples},
  month  = {October},
  year   = {2025},
  url    = {https://github.com/erelbng/ros-examples}
}
```

## Contact

For technical support and other questions, contact [eric.elbing@htwk-leipzig.de](mailto:eric.elbing@htwk-leipzig.de) or [open an issue](https://github.com/erelbng/ros-examples/issues).
