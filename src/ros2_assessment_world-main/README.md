# PDE4430 - Assessment World
### MSc. Robotics, Middlesex University Dubai
**2025 - 2026**

---

## Overview

This package provides a Gazebo Harmonic simulation environment for the PDE4430 Robotics coursework assessment. The assessment world consists of an 8m × 8m enclosed arena with obstacles and randomly spawned spheres that need to be collected and placed into designated pen areas.

### What's Included

- **Assessment World**: An 8m × 8m enclosed environment with:
  - Four perimeter walls (2m height)
  - Two pen areas (approximately 80cm × 80cm each) located at the top corners
  - Nine cylindrical obstacles of varying sizes (0.2m to 0.5m diameter, 1m height)
  - Ground plane with appropriate physics

- **Sphere Spawner**: Automatically spawns three spheres at random locations:
  - Small sphere (0.1m radius) - Red
  - Medium sphere (0.2m radius) - Green
  - Large sphere (0.3m radius) - Blue

## Assessment Task

Your objective is to:
1. Spawn your own mobile robot into this world
2. Navigate the environment while avoiding obstacles
3. Collect the three spheres
4. Transport and place them into one or both of the pen areas

You may use teleoperation, autonomous navigation (SLAM + Nav2), or a combination of both approaches.

---

## Installation & Setup

### Prerequisites
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim)
- ros_gz packages (bridge, sim, interfaces)

### Building the Package

```bash
cd ~/ros2  # or your workspace directory
colcon build --packages-select assessment_world
source install/setup.bash
```

---

## Usage

### Option 1: Launch Everything Together (Recommended)

Launch the complete assessment environment with automatic sphere spawning:

```bash
ros2 launch assessment_world assessment_complete.launch.py
```

This will:
- Start Gazebo with the assessment world
- Wait 5 seconds for initialization
- Automatically spawn the three spheres at random positions

### Option 2: Launch Components Separately

If you need more control over the spawning process (especially if planning to do SLAM without the spheres):

**Terminal 1** - Launch the world:
```bash
ros2 launch assessment_world assessment_world.launch.py
```

**Terminal 2** - Spawn the spheres (after world is loaded):
```bash
ros2 launch assessment_world spawn_spheres.launch.py
```

---

## Integrating Your Robot

To add your robot to this assessment world, you have several options:

### Method 1: Modify the Launch File
Create a new launch file that includes both the assessment world and your robot:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include assessment world
    assessment_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('assessment_world'),
                'launch',
                'assessment_complete.launch.py'
            ])
        ])
    )
    
    # Include your robot spawner
    # your_robot = ...
    
    return LaunchDescription([
        assessment_world,
        # your_robot,
    ])
```

### Method 2: Spawn After Launch
Launch the assessment world first, then use the ROS 2 spawn service to add your robot at a specific location.

---

## Tips for Completing the Assessment

### Understanding the Environment
- The enclosure is 8m × 8m with the origin at the center (0, 0)
- Pen area is located on the left wall
- Obstacles are static
- Spheres have physics enabled - they will roll if bumped

### Robot Design Considerations
- **Size**: Ensure your robot can fit through the spaces between obstacles
- **Manipulator/Gripper/Pusher**: You'll need a way to pick up or push the spheres
- **Sensors**: Consider using:
  - Lidar for obstacle detection and mapping
  - Camera for sphere detection
  - IMU for orientation tracking

### Navigation Strategies

**Teleoperation Approach:**
- Simple and direct control
- Good for learning the environment
- Requires manual sphere detection and collection
- Tip: Practice navigating around obstacles before attempting to collect spheres

**Autonomous Approach (SLAM + Nav2):**
- Use `slam_toolbox` or `cartographer` to create a map
- Once mapped, use Nav2 for autonomous navigation
- Tip: Create the map first without worrying about spheres, then focus on collection

**Hybrid Approach:**
- Use SLAM to map the environment
- Use teleoperation for precise sphere manipulation
- Use Nav2 to navigate between spheres and pens

### Sphere Collection Strategies
1. **Pushing**: Design a front scoop or pusher to guide spheres
2. **Gripping**: Use a gripper to pick up and carry spheres
3. **Containment**: Create a container that spheres can roll into

### Vision and Detection
- Use color-based detection to identify spheres (red, green, blue)
- Subscribe to camera topics to locate spheres
- Consider using packages like `opencv` or `image_pipeline` for processing

### Important Considerations
- Spheres spawn at random positions each time - your solution should be adaptable
- The spheres have different masses (small: 0.3kg, medium: 0.6kg, large: 1.0kg) - heavier spheres require more force to push
- Collision with obstacles will affect your robot's position - implement recovery behaviors
- Test thoroughly - spheres may behave unpredictably when pushed

### Known Issues
- **Sphere spawning inside obstacles**: Occasionally, a sphere may spawn at a position that intersects with an obstacle, causing it to get stuck. This is a known issue with the random spawn mechanism. If this occurs, simply restart the sphere spawning process by re-running the launch file. The spheres will spawn at new random positions.

---

## Troubleshooting

**Gazebo doesn't start:**
- Ensure Gazebo Harmonic is properly installed
- Check that `gz sim` command works independently
- Verify OGRE rendering engine is available, especially if your Gazebo interface is flickering

**Spheres don't spawn:**
- Make sure the world is fully loaded before running spawn script
- Check that `ros_gz_bridge` is installed
- Verify the spawn service is available: `ros2 service list | grep create`

**Robot spawning issues:**
- Ensure spawn position doesn't overlap with walls or obstacles
- Check that your robot's URDF/SDF is properly formatted
- Verify all required plugins are loaded

---

## Package Structure

```
assessment_world/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── assessment_world.launch.py      # Launch world only
│   ├── spawn_spheres.launch.py         # Spawn spheres only
│   └── assessment_complete.launch.py   # Complete setup
├── scripts/
│   └── spawn_spheres.py               # Sphere spawning node
└── worlds/
    └── assessment.sdf                 # Gazebo world definition
```

---

## Support

For technical issues with this package, check:
- ROS 2 service availability: `ros2 service list`
- Gazebo topics: `gz topic -l`
- Launch file output for error messages

---

**Good luck with your assessment!**