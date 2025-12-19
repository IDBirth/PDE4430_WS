# PDE4431 Coursework 1 — Autonomous Ball Collection Robot (ROS 2 + Gazebo)

**Programme:** MSc Robotics (Middlesex University Dubai)  
**Module:** PDE4431  
**Candidate:** Bilal Baslar (**M01099599**)  
**Workspace:** `PDE4430_WS`  

---

## Abstract

This repository contains a ROS 2 workspace targeting the PDE4431 coursework objective:
design and integrate a mobile robot (URDF) that can navigate an assessment world, locate
and collect spherical objects, and deliver them to a known goal location. The work is
structured around URDF modelling, teleoperation, SLAM-based mapping/localisation, Nav2
navigation, ball detection, and pick-and-place integration.

**Keywords:** ROS 2, Gazebo Harmonic, URDF, SLAM Toolbox, Nav2, autonomous navigation, perception, manipulation

---

## Table of Contents

- [Project Overview](#project-overview)
- [Repository Structure](#repository-structure)
- [Build & Run (Reproducibility)](#build--run-reproducibility)
- [Coursework Plan (Source Documentation)](#coursework-plan-source-documentation)
- [Figures (Placeholders)](#figures-placeholders)
- [Limitations & Next Steps](#limitations--next-steps)
- [References](#references)

---

## Project Overview

### Assessment Objective

Create a robot (URDF) that:
1. Navigates autonomously in a simulated world,
2. Picks up balls (spheres) from the environment, and
3. Delivers them to a known goal area (e.g., a pen).

### Simulation Environment

This workspace includes an assessment world package providing an enclosed arena with obstacles and randomly spawned spheres.

---

## Repository Structure

This repository is organised as a ROS 2 workspace (typical `colcon` layout):

```text
PDE4430_WS/
  build/
  install/
  log/
  src/
    nav2_stack/                   # Nav2 bringup + RViz config + maps/params
    ros2_assessment_world-main/   # Gazebo world + sphere spawner (package: assessment_world)
    My_Robot/                     # (reserved) robot description/manipulation work
```

**Notes**
- Package names (from `package.xml`): `nav2_stack`, `assessment_world`.
- `src/My_Robot/` currently exists as a workspace folder for robot assets (URDF, meshes, launch, etc.).

---

## Build & Run (Reproducibility)

### Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim`)
- `nav2_bringup`
- `slam_toolbox` (for mapping/localisation)
- `ros_gz` bridge packages (for Gazebo ↔ ROS 2 integration), as required by your setup

### Build

```bash
cd PDE4430_WS
colcon build --symlink-install
source install/setup.bash
```

### Launch the Assessment World

Recommended (world + automatic sphere spawning):

```bash
ros2 launch assessment_world assessment_complete.launch.py
```

Or split launch (useful when mapping without spheres):

```bash
# Terminal 1
ros2 launch assessment_world assessment_world.launch.py

# Terminal 2 (after Gazebo is fully loaded)
ros2 launch assessment_world spawn_spheres.launch.py
```

### Launch Nav2 (with RViz)

This workspace provides a lightweight Nav2 bringup wrapper:

```bash
ros2 launch nav2_stack nav2.launch.py
```

Optional: override the map and/or parameters:

```bash
ros2 launch nav2_stack nav2.launch.py \
  map:=$(ros2 pkg prefix --share nav2_stack)/config/map.yaml \
  params_file:=$(ros2 pkg prefix --share nav2_stack)/config/nav2_params.yaml
```

---

## Coursework Plan (Source Documentation)

The following section is included from the coursework plan documentation page:
`https://bilal-baslar-mdx.blogspot.com/p/pde4431-cw1-plan.html`

### Verbatim Plan Text (Structured)

This is the plan and approach for solving the PDE4431 Coursework by Bilal Baslar (M01099599).

#### Problem Breakdown
**Overview:** Create a robot (URDF) that navigates autonomously, picks up balls from the world, and delivers them to the known goal.

#### Problem Identification & Solving

**URDF Problem**
- Making body and gripping system
- `base_link`
- Joint limitations
- Camera setup
- Caster wheel setup
- Differential-drive setup
- LiDAR setup
- Create camera publisher

**Teleoperate problem**
- Use the pre-built teleop with `cmd_vel`

**SLAM problem**
- Create map
- Use to create a map: `async_slam_toolbox_node`
- Use to localise: `localization_slam_toolbox_node`

**Nav2 problem**
- To be studied

**Ball Detection Problem**
- Create (TBD)

**Pick and Place problem**
- (TBD)

**Launch structure problem**
- (TBD)

**Documentation Problem**
- (TBD)

#### URDF Plan
- `URDF_1`: (TBD)
- `URDF_2`: (TBD)
- Added camera
- Added arms to hold balls; arms move ~30° to close, compatible with all ball sizes

---

## Figures (Placeholders)

Add figures into `docs/figures/` and update the links below. Leave captions intact for report-quality documentation.

### Figure 1 — System Architecture

![Figure 1: System-level architecture showing Gazebo world, ROS 2 nodes, Nav2, SLAM toolbox, and perception/manipulation pipelines.](docs/figures/fig01_system_architecture.png)


### Figure 2 — Robot URDF Overview

![Figure 2: URDF model overview (links/joints, sensors, gripper/arms).](docs/figures/fig02_urdf_overview.png)


### Figure 3 — SLAM Map Output

![Figure 3: Generated occupancy grid map and localisation status.](docs/figures/fig03_slam_map.png)


### Figure 4 — Navigation Stack in RViz

![Figure 4: Nav2 costmaps, global plan, and goal execution visualised in RViz.](docs/figures/fig04_nav2_rviz.png)


### Figure 5 — Ball Detection

![Figure 5: Example detection output (camera view + detected spheres).](docs/figures/fig05_ball_detection.png)


### Figure 6 — Pick-and-Place / Delivery

![Figure 6: Ball collection and delivery sequence to the goal pen.](docs/figures/fig06_pick_place.png)

---

## Limitations & Next Steps

- `src/My_Robot/` is currently a placeholder directory; add your robot description package (URDF/Xacro, meshes, plugins, and launch files).
- The plan items marked **(TBD)** are intentionally left open to be filled with implementation details (packages, topics, parameters, and evaluation results).
- Consider adding a dedicated `docs/` report (PDF) and a reproducible experiment log (commands, seeds, environment versions).

---

## Compliance (Gazebo Ground Truth)

The running system does **not** use Gazebo model pose/state topics to locate spheres. It relies on standard robot
sensors and ROS interfaces. Gazebo ground‑truth topics are reserved for debugging/verification only.

---

## References

- PDE4431 CW1 plan page: `https://bilal-baslar-mdx.blogspot.com/p/pde4431-cw1-plan.html`
- Assessment world package documentation: `src/ros2_assessment_world-main/README.md`
