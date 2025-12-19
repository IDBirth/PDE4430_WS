# PDE4430 Coursework — Mobile Robot for Ball Collection (ROS 2 Jazzy + Gazebo Harmonic)

**Programme:** MSc Robotics (Middlesex University Dubai)  
**Module:** PDE4430  
**Candidate:** Bilal Baslar (M01099599)  
**Workspace:** `PDE4430_WS`

---

## Abstract

This repository contains a ROS 2 workspace for the PDE4430 mobile robotics coursework. The project delivers a
Gazebo Harmonic assessment world and multiple iterations of a differential-drive robot model (URDF/Xacro + meshes)
with SLAM-ready launch files. The workspace is structured for reproducibility and extensions toward perception and
manipulation for ball collection.

**Keywords:** ROS 2 Jazzy, Gazebo Harmonic, URDF/Xacro, ros_gz, SLAM Toolbox, teleoperation

---

## Table of Contents

- [Project Overview](#project-overview)
- [Repository Structure](#repository-structure)
- [Build & Run (Reproducibility)](#build--run-reproducibility)
- [Packages](#packages)
- [Topics and Frames](#topics-and-frames)
- [Compliance (Gazebo Ground Truth)](#compliance-gazebo-ground-truth)
- [Figures (Placeholders)](#figures-placeholders)
- [Limitations & Next Steps](#limitations--next-steps)
- [References](#references)

---

## Project Overview

### Assessment Objective

Create a robot that:
1. Navigates in the assessment world,
2. Collects spheres, and
3. Delivers them to the pen area.

### Simulation Environment

The `assessment_world` package provides an 8m x 8m arena with obstacles and randomly spawned spheres.

---

## Coursework Submision

Blog Link: `https://bilal-baslar-mdx.blogspot.com/p/pde4431-cw1-plan.html`
Github: 'https://github.com/IDBirth/PDE4430_WS/tree/testing?tab=readme-ov-file#compliance-gazebo-ground-truth '
Youtube: "https://youtu.be/FiY-CyodFZo"

This is the plan and approach for solving the PDE4431 Coursework by Bilal Baslar (M01099599).

### Problem Breakdown

**Overview:** Create a robot (URDF) that navigates autonomously, picks up balls from the world, and delivers them to the known goal.

**Problem Identification & Solving**
- URDF problem:
  - Making body and gripping system
  - `base_link`
  - Joint limitations for `left_arm` and `right_arm`
  - Camera setup
  - Caster wheel setup
  - Diff-drive setup
  - Lidar setup
  - Create camera publisher
- Teleoperate problem:
  - Use the pre-built teleop to publish to `/cmd_vel`
- SLAM problem:
  - Create a map of the provided course map (store it in the `map/` folder)
  - Use `async_slam_toolbox_node` to create a map
  - Use `localization_slam_toolbox_node` to localise if required
  - Save the map and config for RViz
- Nav2 problem:
  - Navigate to the goal position
  - Go to a ball pose while avoiding collisions
  - Hardcode drop point in the map
- Ball detection problem + ball pose:
  - Create a subscriber to `steam_node`
  - Run x frames/s through an OpenCV or YOLO model
    - https://www.youtube.com/watch?v=5yx5zpu6nk8
  - Compute position data
  - Output pose of the ball relative to the camera
- Autonomous pick and place problem:
  - Detect ball in zone
  - Call `Close_Grip` service
  - Articulate gripper to close and open
  - Call `Open_Grip` service
- Launch structure problem:
  - Launch world (wait 5 sec)
  - Launch robot
  - Launch ball
  - Launch detection nodes
  - Launch RViz with config
  - Launch GUI with 3 buttons: Start, Reset, Pause Robot
- Documentation problem:
  - Save all here: https://bilal-baslar-mdx.blogspot.com/p/pde4431-cw1-plan.html

### Plan Approach

**URDF Plan**
- Use Fusion with the Fusion2URDF script to create a base URDF file
- Use the install script and edit the Xacro
- Edit the URDF to add lidar, diff-drive, camera, arms (`left_arm` and `right_arm`), and caster wheel
- Construct the TF tree correctly
- Ensure wheel separation is correct

**Iteration**
- URDF_1: https://a360.co/49gF3uo
- URDF_2: https://a360.co/4s2ivoM

**Issues Faced**
- Biggest ball is too big

**Between V1 and V2 Changes**
- Added camera
- Added arms to hold balls; arms move 30 degrees to close and work for all ball sizes
- Added caster wheel

**Important Details**
- Wheel spacing: 776.4 mm (0.776 m)
- Arms closed at 30 degrees with the biggest ball
- Arms closed at 45 degrees with the smallest ball

**TF Tree for My_Robot**
- `base_link`:
  - `lidar`
  - `camera`
  - `right_wheel`
  - `left_wheel`
  - `right_arm`
  - `left_arm`
  - `caster`

**Diff-drive setup**
- https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller

**Lidar setup**
- Needs to be like the previous

**Camera**
- https://youtu.be/jgXeIXrckBc
- https://www.youtube.com/watch?v=5yx5zpu6nk8
- https://chatgpt.com/share/694367a3-eb60-800d-a04e-d29e33882437

**Caster wheel**
- Decreased the friction to 0.1:
  - `<mu1>0.1</mu1>`
  - `<mu2>0.1</mu2>`

**Major issue after 1st test on My_Robotv3**
- Base link is off
- Arms move while driving, almost every link is off
- Wheels aren't able to spin freely
- Success: camera works and publishes to `/image_raw/image`

**Teleop Plan**
- Use the built-in teleop node

**SLAM**
- Map of the provided course map
- Use `async_slam_toolbox_node` to create a map
- Use `localization_slam_toolbox_node` to localise if required
- Save the map and config for RViz

**Showcase moving using Teleop**
- \"Acceptable Level\" submission: https://youtu.be/g7YHDBDMrE0

---

## Repository Structure

```text
PDE4430_WS/
  back/                         # Reference templates and earlier URDF variants
  build/
  install/
  log/
  docs/                         # Coursework brief, worksheet, and artifacts
  src/
    My_Robot/                   # my_robot_description (baseline robot)
    My_Robotv2/                 # v2 robot description package
    My_Robotv3/                 # v3 robot description package
    my_bot_perception/          # ball detection + goal transform nodes
    nav2_stack/                 # Nav2 bringup wrapper
    ros2_assessment_world-main/ # assessment_world (Gazebo environment)
```

---

## Build & Run (Reproducibility)

### Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim`)
- `ros_gz` packages (`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`)
- `xacro`
- `slam_toolbox`
- `cv_bridge` + OpenCV (for `my_bot_perception`)

### Build

```bash
cd ~/PDE4430_WS
colcon build --symlink-install
source install/setup.bash
```

### World Only

```bash
ros2 launch assessment_world assessment_world.launch.py
```

### World + Spheres

```bash
ros2 launch assessment_world assessment_complete.launch.py
```

### Baseline Robot (with SLAM + RViz)

```bash
ros2 launch my_robot_description start.launch.py
```

### My_Robotv3 (Gazebo + spawn + bridge + arm control)

```bash
ros2 launch my_robot_v3_description gazebo.launch.py
```

### My_Robotv3 (Gazebo + RViz combined)

```bash
ros2 launch my_robot_v3_description URDF_Test.launch.py
```

### My_Robotv3 Full Start (Assessment world + robot + bridge + teleop + perception)

```bash
ros2 launch my_robot_v3_description start.launch.py
```

### Ball Perception (standalone)

```bash
ros2 launch my_bot_perception ball_perception_with_goal.launch.py
```

### Arm Control (open/close)

The arm controller node is started by the v3 launch files and exposes two services:
- `/arms/open` (opens both arms)
- `/arms/close` (closes both arms to +/- 30 deg)

Examples:

```bash
ros2 service call /arms/open std_srvs/srv/Trigger {}
ros2 service call /arms/close std_srvs/srv/Trigger {}
```

You can also drive the joints directly:

```bash
ros2 topic pub /left_arm/cmd_pos std_msgs/msg/Float64 "{data: 0.2}"
ros2 topic pub /right_arm/cmd_pos std_msgs/msg/Float64 "{data: -0.2}"
```

---

## Packages

### 1) `my_robot_description`
**Location:** `src/My_Robot`

Baseline robot description package with:
- URDF/Xacro model + meshes
- Gazebo Harmonic plugins (diff drive + lidar)
- Launch files for RViz, Gazebo spawn, and SLAM toolbox

Key launch files:
- `display.launch.py`
- `gazebo.launch.py`
- `start.launch.py`
- `slam_gazebo.launch.py`

### 2) `my_robot_cam_udrf_description` (v2)
**Location:** `src/My_Robotv2`

Second iteration of the robot description (camera + arms + updated meshes).

### 3) `my_robot_v3_description`
**Location:** `src/My_Robotv3`

Third iteration integrating v1 control/bridge with v2 body/arms/camera and updated launch files.
Includes custom arm control services and Gazebo tuning.

Key launch files:
- `start.launch.py` (assessment world + spawn + bridge + teleop + perception with 5s delay)
- `gazebo.launch.py` (Gazebo + spawn + bridge + arm control)
- `URDF_Test.launch.py` (Gazebo + RViz combined)

### 4) `assessment_world`
**Location:** `src/ros2_assessment_world-main`

Gazebo Harmonic assessment arena with obstacles and sphere spawner.

### 5) `my_bot_perception`
**Location:** `src/my_bot_perception`

Ball perception package with:
- `circle_ball_node` (combined detector + visualizer)
- `ball_goal_transformer` (TF2 pose transform + standoff goal)
- `ball_perception_with_goal.launch.py`

### 6) `nav2_stack`
**Location:** `src/nav2_stack`

Lightweight Nav2 bringup wrapper and RViz configuration.

---

## Topics and Frames

Typical bridged topics:
- `/cmd_vel` (ROS -> Gazebo)
- `/odom` (Gazebo -> ROS)
- `/scan` (Gazebo -> ROS)
- `/tf`, `/joint_states`, `/clock`

Common frames:
- `base_link`, `lidar_1`, `odom`, `map`

Perception topics:
- `/camera/image_raw`
- `/camera/camera_info`
- `/pose_ball`
- `/camera/image_ball`
- `/goal_pose`

---

## Compliance (Gazebo Ground Truth)

The running system does **not** use Gazebo model pose/state topics to locate spheres. It relies on standard robot
sensors and ROS interfaces. Gazebo ground‑truth topics are reserved for debugging/verification only.

---

## Figures (Placeholders)

Add figures into `docs/figures/` and update the links below.

- `docs/figures/fig01_system_architecture.png`
- `docs/figures/fig02_urdf_overview.png`
- `docs/figures/fig03_slam_map.png`
- `docs/figures/fig04_nav2_rviz.png`
- `docs/figures/fig05_ball_detection.png`
- `docs/figures/fig06_pick_place.png`

---

## Limitations & Next Steps

- Ball detection and goal generation are available, but closed-loop navigation/pickup behavior is not fully integrated.
- ros2_control controllers for arms are not implemented.
- Add evaluation artifacts (maps, screenshots, RQT graph, video link) to strengthen documentation.

---

## References

- Coursework brief: `docs/DBI_PDE4430_CW.pdf`
- Worksheet: `docs/CW_Worksheet.pdf`
- Assessment world docs: `src/ros2_assessment_world-main/README.md`
- Robot package docs: `src/My_Robot/README.md`
