# MaRs-AutoNav

> **Branch:** `tharanesh` (RTAB-Map SLAM, Noise Filtering & Parameter Tuning)

Autonomous Navigation, Simulation, and SLAM stack for a 4-wheel independent swerve-drive rover in Gazebo Sim using **ROS 2 Humble**.

---

## 📋 Overview

**MaRs-AutoNav** (`autonavtest`) provides a full simulation and mapping pipeline:
* **Rover Platform:** 4-wheel independent swerve-drive rover featuring suspension, individual steering position controllers, and wheel drive velocity controllers.
* **Kinematics Engine:** Custom Python node (`swerve_controller.py`) translating `/cmd_vel` into coordinated 4-wheel swerve steering angles and drive velocities.
* **Sensor Simulation & Bridge:** Bridges Gazebo Sim (Ignition) sensor data to ROS 2 topics:
  * RGB-D Depth Camera (color, depth, and camera info)
  * IMU (`/imu/data`)
  * GPS / NavSat (`/gps/fix`)
  * Odometry (`/model/ares_nova/odometry`)
* **Sensor Fusion (EKF):** `robot_localization` EKF node fusing wheel odometry and IMU into `/odometry/local`.
* **SLAM & Mapping (RTAB-Map):** Dedicated headless RTAB-Map node generating a 2D occupancy grid (`/rtabmap/map`) from RGB-D data, fine-tuned for complex 3D mesh terrains.
* **RViz2 Visualization:** Integrated display profile showing the rover model, live camera stream, and occupancy grid.

---

## 🛠 Prerequisites & Dependencies

* **OS:** Ubuntu 22.04 LTS
* **ROS Distro:** [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)
* **Simulator:** Gazebo Sim (Ignition Gazebo) with `ros_gz` packages

### Install Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-robot-localization \
  ros-humble-rtabmap-launch \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-viz \
  ros-humble-teleop-twist-keyboard \
  ros-humble-xacro
```

---

## 🚀 Build Instructions

1. **Clone the repository and switch to the `tharanesh` branch:**

   * **Option A: Clone this branch directly:**
     ```bash
     git clone -b tharanesh https://github.com/saitejapcv/MaRs-AutoNav.git
     cd MaRs-AutoNav/ros_ws
     ```

   * **Option B: If you already cloned the repository (switch from `main`):**
     ```bash
     cd MaRs-AutoNav
     git checkout tharanesh
     cd ros_ws
     ```

2. **Build the workspace:**
   ```bash
   colcon build --packages-select autonavtest
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

---

## 🎮 How to Run

To run the complete simulation, mapping, and teleoperation system, open separate terminal windows:

### Terminal 1: Launch Gazebo Simulation & EKF
Starts Gazebo Sim, loads the terrain, spawns the rover, activates `ros2_control` spawners, launches the swerve brain, and starts EKF localization:
```bash
source /opt/ros/humble/setup.bash
source ~/MaRs-AutoNav/ros_ws/install/setup.bash
ros2 launch autonavtest launch_sim.launch.py
```

### Terminal 2: Launch RTAB-Map SLAM & RViz2
Starts RTAB-Map headlessly (standalone GUI suppressed) and launches RViz2 with the pre-configured layout:
```bash
source /opt/ros/humble/setup.bash
source ~/MaRs-AutoNav/ros_ws/install/setup.bash
ros2 launch autonavtest rtabmap.launch.py
```

### Terminal 3: Teleoperate the Rover
Control the rover manually using your keyboard:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
* Use `i`, `,`, `j`, `l` to drive linear X/Y and steer angular Z.
* Press `k` to stop.

---

## 🗺 RTAB-Map Grid Parameter Tuning

The mapping pipeline is pre-configured in `rtabmap.launch.py` with custom grid parameters tuned specifically for the uneven 3D STL mesh terrain (`map2.stl`):

```bash
args:="--delete_db_on_start \
       --Grid/RayTracing false \
       --Grid/MaxGroundAngle 29 \
       --Grid/RangeMax 5.0 \
       --Grid/RangeMin 0.4 \
       --Grid/FlatObstacleDetected false \
       --Grid/NormalK 30 \
       --Grid/NoiseFilteringRadius 0.15 \
       --Grid/NoiseFilteringMinNeighbors 8 \
       --Grid/MinClusterSize 30 \
       --GridGlobal/Eroded true \
       --Grid/MinGroundHeight -0.20"
```

### Key Parameter Explanations:
* **`--Grid/MaxGroundAngle 29`:** Safely ignores flat ground mesh seams (max angle $27.37^\circ$) while reliably detecting sloped obstacles, boulders, and trenches ($>30^\circ$).
* **`--Grid/MinGroundHeight -0.20`:** Prevents surfaces deeper than 20 cm below the rover's wheels from being classified as traversable ground, flagging deep pits as hazards.
* **`--Grid/RangeMax 5.0`:** Eliminates far-field depth quantization striations and edge zebra stripes.
* **`--Grid/RangeMin 0.4`:** Ignores near-field camera artifacts caused by the rover's own bumper and front wheels.
* **`--Grid/FlatObstacleDetected false`:** Forces flat surfaces meeting ground criteria to be treated strictly as ground.
* **`--Grid/NormalK 30`:** Computes surface normals across 30 neighbor points to smooth out depth camera jitter.
* **`--Grid/NoiseFilteringRadius 0.15` & `--Grid/NoiseFilteringMinNeighbors 8`:** Removes isolated speckles and sensor noise.
* **`--Grid/MinClusterSize 30`:** Discards obstacle clusters smaller than 30 points.
* **`--GridGlobal/Eroded true`:** Applies 1-cell morphological erosion to strip single-pixel noise from the 2D map.

---

## 📁 Package Structure

```text
MaRs-AutoNav/
└── ros_ws/
    └── src/
        ├── CMakeLists.txt                # Package build rules (installs launch, config, rviz, worlds, description)
        ├── package.xml                   # Package dependencies
        ├── config/
        │   ├── ekf.yaml                  # robot_localization EKF config
        │   └── ros2_controllers.yaml     # Steering and drive controller configs
        ├── description/
        │   ├── chassis.xacro             # Chassis, camera, and GPS link definitions
        │   ├── rover_properties.xacro    # CAD dimensions and wheel geometry
        │   ├── rover.urdf.xacro          # Main URDF assembler & Gazebo plugins
        │   ├── suspension.xacro          # Suspension arm definitions
        │   └── wheel.xacro               # Swerve steer & drive wheel macro
        ├── launch/
        │   ├── launch_sim.launch.py      # Gazebo world, rover spawn, bridge, EKF, swerve node
        │   └── rtabmap.launch.py         # Headless RTAB-Map SLAM + RViz2
        ├── rviz/
        │   └── default.rviz              # Pre-configured RViz display profile
        ├── scripts/
        │   └── swerve_controller.py      # 4-wheel swerve inverse kinematics node
        └── worlds/
            ├── world.sdf                 # Gazebo simulation world definition
            └── map2.stl                  # 3D terrain mesh
```

---

## 📡 Core Topics

| Topic | Type | Source | Purpose |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Teleop / Nav2 | Velocity commands |
| `/steering_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | `swerve_controller.py` | Steering joint angles |
| `/drive_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | `swerve_controller.py` | Wheel drive speeds |
| `/odometry/local` | `nav_msgs/msg/Odometry` | `robot_localization` | EKF filtered odometry |
| `/depth_camera/image_raw/image` | `sensor_msgs/msg/Image` | `ros_gz_bridge` | RGB camera stream |
| `/depth_camera/image_raw/depth_image` | `sensor_msgs/msg/Image` | `ros_gz_bridge` | Depth camera stream |
| `/depth_camera/image_raw/camera_info` | `sensor_msgs/msg/CameraInfo` | `ros_gz_bridge` | Camera intrinsics |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | `rtabmap_slam` | 2D SLAM occupancy grid |
| `/imu/data` | `sensor_msgs/msg/Imu` | `ros_gz_bridge` | Rover IMU data |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | `ros_gz_bridge` | NavSat GPS position |
