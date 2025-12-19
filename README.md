# PDE4430 Coursework — Mobile Robot for Ball Collection (ROS 2 Jazzy + Gazebo Harmonic)

**Programme:** MSc Robotics (Middlesex University Dubai)  
**Module:** PDE4430  
**Candidate:** Bilal Baslar (M01099599)  
**Workspace:** `PDE4430_WS`

---

## Abstract

This repository contains a ROS 2 workspace for the PDE4430 mobile robotics coursework. The project delivers a
Gazebo Harmonic simulation environment and a differential-drive robot model (URDF/Xacro + meshes) that can be
spawned into the assessment world, teleoperated, and used as a baseline for mapping and autonomous navigation.
The system is structured for clear reproducibility and extensibility toward ball detection and manipulation.

**Keywords:** ROS 2 Jazzy, Gazebo Harmonic, URDF/Xacro, ros_gz, SLAM Toolbox, teleoperation
but
---

## Repository Structure (Actual)

```
PDE4430_WS/
  build/
  install/
  log/
  docs/                         # Coursework brief and worksheet
    CW_Worksheet.pdf
    DBI_PDE4430_CW.pdf
  src/
    My_Robot/                   # Robot description package (my_robot_description)
    ros2_assessment_world-main/ # Assessment world package (assessment_world)
```

---

## Packages

### 1) `my_robot_description`
**Location:** `src/My_Robot`

A ROS 2 (ament_python) robot description package containing:
- URDF/Xacro model with meshes (`urdf/`, `meshes/`)
- Gazebo Harmonic plugins (diff drive + lidar) via `urdf/My_Robot.gazebo`
- Launch files for RViz visualization, Gazebo spawning, and SLAM toolbox integration

Key launch files:
- `display.launch.py` — RViz visualization only
- `gazebo.launch.py` — empty Gazebo world + robot spawn + bridge
- `start.launch.py` — assessment world + robot spawn + bridge + SLAM + RViz + teleop
- `slam_gazebo.launch.py` — assessment world + SLAM + RViz + teleop

### 2) `assessment_world`
**Location:** `src/ros2_assessment_world-main`

Gazebo Harmonic assessment arena with:
- 8m x 8m enclosed world, obstacles, and pen areas
- Randomized spawning of three spheres (small/medium/large)
- Launch files for world-only or world+spawner

---

## Build

```bash
cd ~/PDE4430_WS
colcon build --symlink-install
source install/setup.bash
```

---

## Run (Quick Start)

### 1) Assessment world + robot + SLAM + RViz (recommended)

```bash
ros2 launch my_robot_description start.launch.py
```

### 2) World only

```bash
ros2 launch assessment_world assessment_world.launch.py
```

### 3) World + spheres

```bash
ros2 launch assessment_world assessment_complete.launch.py
```

### 4) Robot in empty Gazebo world

```bash
ros2 launch my_robot_description gazebo.launch.py
```

### 5) RViz only (no Gazebo)

```bash
ros2 launch my_robot_description display.launch.py
```

---

## Robot Capabilities (Current Baseline)

- Differential drive base with Gazebo diff-drive plugin
- GPU lidar sensor publishing `/scan`
- ROS <-> Gazebo bridge for `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, `/scan`, `/clock`
- SLAM Toolbox launch integration for mapping experiments

---

## Topics and Frames (Core)

**ROS topics (bridged):**
- `/cmd_vel` (ROS -> Gazebo)
- `/odom` (Gazebo -> ROS)
- `/scan` (Gazebo -> ROS)
- `/tf`, `/joint_states`, `/clock`

**Frames:**
- `base_link`, `lidar_1`, `odom`, `map`

---

## Coursework Alignment

The workspace supports the required coursework milestones described in `docs/`:
- URDF/Xacro model with inertial, visual, and collision elements
- Gazebo simulation with sensor plugins
- Teleoperation readiness via `/cmd_vel`
- SLAM integration for mapping
- A documented, reproducible ROS 2 workspace structure

The design is intended to demonstrate feasibility of moving spheres into the pen area, with clear extension points
for perception (ball detection) and manipulation.

---

## Documentation

- Coursework worksheet and brief: `docs/CW_Worksheet.pdf`, `docs/DBI_PDE4430_CW.pdf`
- Assessment world details: `src/ros2_assessment_world-main/README.md`
- Robot package details: `src/My_Robot/README.md`

---

## Troubleshooting

- **Robot not spawning:** wait ~5 seconds after Gazebo startup; check `ros_gz_sim create` output.
- **No lidar data:** confirm `/scan` topic and that SLAM parameters use `scan_topic: /scan`.
- **Xacro include errors:** ensure the workspace is built and `source install/setup.bash` is run.
- **Teleop window not opening:** `slam_gazebo.launch.py` uses `xterm`. Install it or remove the prefix.

---

## Submission Checklist (GitHub)

- Code repository with clear commit history
- README with build/run instructions (this file)
- Video demonstration (5 minutes) showing robot operation and task feasibility
- Evidence of structured design choices and extension toward autonomous strategies

---

## License

See individual package `package.xml` files. The assessment world package is licensed under Apache-2.0.

