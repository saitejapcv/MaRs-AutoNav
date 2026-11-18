# MaRs AutoNav — IRC Rover Autonomous Navigation Stack

Autonomous navigation software stack for the IRC Martian rover, featuring a **4-Wheel Independent Steering & Drive (4WIS/4WID) Holonomic Swerve** drivetrain, multi-sensor EKF state estimation, RTAB-Map 3D surface normal SLAM, and ROS 2 Nav2 path planning.

---

## Quick Start

### 1. Build the Workspace
```bash
cd ~/Desktop/MaRs-AutoNav/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. Launch the Simulation
```bash
# Optional: clean up lingering simulator processes
killall -9 ruby ign gz_sim parameter_bridge rviz2 2>/dev/null

source install/setup.bash
ros2 launch autonavtest launch_sim.launch.py
```
*(Gazebo, ros2_control, EKF, RTAB-Map, RViz2, and Nav2 will start sequentially).*

### 3. Send Goals
* **RViz2:** Click **Nav2 Goal** (hotkey `g`) and drag a goal pose on the map.
* **Teleop:** In a new terminal (`source install/setup.bash`), run:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

---

## File Structure & Descriptions

### `ros_ws/src/` (Package: `autonavtest`)

| File / Folder | Purpose |
| :--- | :--- |
| **`CMakeLists.txt`** | Package build rules and install directories for launch, config, description, worlds, and scripts. |
| **`package.xml`** | Package dependencies (Nav2, RTAB-Map, `robot_localization`, `ros2_control`, etc.). |
| **`config/nav2_params.yaml`** | Nav2 stack configuration: holonomic DWB controller, Navfn global planner, 2D/3D costmap layers, lifecycle manager, and Groot2 telemetry ports. |
| **`config/ekf.yaml`** | `robot_localization` EKF settings fusing wheel odometry (50 Hz), IMU (100 Hz), and RTK-GPS into `/odometry/filtered`. |
| **`config/ros2_controllers.yaml`** | `ros2_control` hardware interface configuration for joint state broadcasting, drive velocity, and steer position controllers. |
| **`config/autonavtest.rviz`** | Pre-configured RViz2 display for robot model, TF tree, point clouds, global/local costmaps, and path tracking. |
| **`description/rover.urdf.xacro`** | Top-level robot model assembling chassis, rocker-bogie suspension, swerve wheels, sensors (ZED 2i, IMU, GPS), and Gazebo plugins. |
| **`description/chassis.xacro`** | Main structural frame geometry, collision bounds, and mass properties. |
| **`description/suspension.xacro`** | Rocker-bogie suspension linkages and passive pivot joints. |
| **`description/wheel.xacro`** | Swerve wheel module macro defining independent steering pivots and drive wheels. |
| **`description/rover_properties.xacro`**| Dimension constants, materials, and inertial tensor macros. |
| **`description/preview.urdf`** | Fully expanded, standalone URDF for VS Code URDF 3D visual preview. |
| **`launch/launch_sim.launch.py`** | Master launch file: stages Gazebo Sim, bridge, `ros2_control`, swerve controller, EKF, RTAB-Map, RViz2, and Nav2. |
| **`launch/navigation.launch.py`** | Nav2 lifecycle launcher bringing up planner, controller, smoother, and behavior servers. |
| **`scripts/swerve_controller.py`** | 4WIS/4WID kinematics node: converts `/cmd_vel` to wheel angles/speeds with $\pm 90^\circ$ angle optimization, and publishes `/odom`. |
| **`worlds/world.sdf`** | Gazebo simulation world containing lighting, physics properties, and the Martian terrain mesh. |
| **`worlds/map2.stl`** | 3D mesh of the IRC Sproscape arena terrain (craters, slopes, and boulders). |
| **`worlds/base_cone.dae`** | Visual waypoint marker mesh for simulation targets. |
| **`screenshots/urdf_preview.png`** | Reference rendering of the rover 3D URDF model. |

---

### Root Documentation Files

| File | Description |
| :--- | :--- |
| **`Nav2_Implementation_Plan.docx / .pdf / .html`** | Technical architecture documentation for the Nav2 integration, behavior trees, and costmap configurations. |
| **`frames_*.pdf`** | Validated ROS 2 TF transform tree diagram (`map` $\to$ `odom` $\to$ `base_footprint`). |
