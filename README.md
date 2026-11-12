# Caterpillar Autonomy Challenge (Shaastra 2027) — Rover Autonomy Stack

Autonomous navigation and mission execution stack for the 4WD skid-steer lunar excavation rover competing in the **Shaastra 2027 Caterpillar Autonomy Challenge**.

---

## Quickstart

```bash
# 1. Build the workspace
cd /home/yashvant/Desktop/MaRs-Caterpillar
colcon build
source install/setup.bash

# 2. Launch complete simulation, perception, localization & Nav2 stack
ros2 launch caterpillar_challenge cat_simulation.launch.py
```

In RViz, use the **2D Goal Pose** tool (or press `G`) to click any goal in the arena; the rover will autonomously plan and navigate around boulders and craters.

---

## File Overview

### 1. Launch Files (`src/launch/`)
* **`cat_simulation.launch.py`**: Master launch file bringing up Ignition Gazebo arena, robot state publisher, controllers, localization, perception, Nav2, and RViz2.
* **`cat_navigation.launch.py`**: Nav2 stack bringup (NavfnPlanner, RegulatedPurePursuitController, costmaps, lifecycle manager, and `cmd_vel_relay`).
* **`cat_localization.launch.py`**: State estimation bringup (`robot_localization` EKF node + `aruco_localizer`).
* **`cat_perception.launch.py`**: Launches `terrain_hazard_filter` for crater/boulder detection and perimeter keepout generation.
* **`view_rover.launch.py`**: Standalone lightweight visualizer for inspecting rover URDF and TF frames in RViz.

### 2. Python Nodes (`src/scripts/`)
* **`aruco_localizer.py`**: Detects Starting Zone wall ArUco markers via OpenCV PnP, publishes initial pose lock, and broadcasts continuous `map -> odom` TF at 30 Hz.
* **`terrain_hazard_filter.py`**: Map-referenced hazard classifier; segments boulders ($>+10\text{ cm}$) and craters ($<-10\text{ cm}$), strips physical walls (Rule 3/4 compliance), and generates virtual perimeter keepouts.
* **`skid_steer_controller.py`**: 4WD skid-steer drive node converting `/cmd_vel` to paired wheel velocities with sand slip compensation.
* **`cmd_vel_relay.py`**: Bridges Nav2 `/cmd_vel` output to `diff_drive_base_controller/cmd_vel_unstamped`.
* **`teleop_keyboard.py`**: Keyboard teleoperation script for manual rover drive testing.

### 3. Configuration (`src/config/`)
* **`nav2_params.yaml`**: Nav2 configuration (NavfnPlanner, RegulatedPurePursuitController, global & local costmaps tuned for loose sand).
* **`ekf.yaml`**: `robot_localization` EKF fusing wheel speed ($v_x$) with IMU angular velocity ($\omega_z$) and acceleration ($a_x$); compass/magnetometer strictly disabled.
* **`ros2_controllers.yaml`**: `ros2_control` configuration for `joint_state_broadcaster` and `diff_drive_base_controller`.
* **`cat_rover.rviz`**: Pre-configured RViz layout displaying arena costmaps, point clouds, TF tree, and the 2D Goal Pose tool.

### 4. Robot Description (`src/description/`)
* **`cat_rover.urdf.xacro`**: Complete rover URDF/Xacro with 4 cleated wheels, compliant prismatic droop suspension, ZED 2i stereo camera, and 6-DOF IMU.
* **`cat_rover_properties.xacro`**: Physical constants (dimensions, masses, suspension limits, wheel geometry, and materials).
* **`meshes/cleated_wheel.obj`**: 3D mesh of the cleated sand wheels.
* **`generate_wheel_mesh.py`**: Procedural script generating `cleated_wheel.obj`.

### 5. World & Simulation Assets (`src/worlds/`)
* **`cat_arena.sdf`**: Ignition Gazebo world for the $9\text{ m} \times 5\text{ m}$ sand arena with 3 ArUco wall markers and enclosure walls.
* **`models/cat_terrain.obj`**: 3D terrain mesh containing 4 deep, wide craters.
* **`models/boulder_1.obj` – `4.obj`**: 3D meshes of positive boulder obstacles.
* **`textures/cat_sand_diffuse.png`**: High-resolution sand surface texture with crater shading.
* **`textures/aruco_tag_0.png` – `2.png`**: ArUco `DICT_4X4_50` marker textures mounted on Starting Zone walls.
* **`generate_arena_assets.py`**: Procedural generation script for arena terrain and boulders.

### 6. Behavior Tree (`src/behavior_trees/`)
* **`cat_mission_tree.xml`**: BehaviorTree.CPP v4 XML visualizable in **Groot / Groot2** representing the complete 2-cycle hands-free mission (Start Lock $\to$ Outbound Slalom $\to$ Excavate $\to$ Return Slalom $\to$ Relocalize $\to$ Berm Dump).
