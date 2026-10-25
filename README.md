# MaRs-AutoNav

> **Branch:** `tharanesh` (Autonomous Navigation with Nav2, RTAB-Map SLAM & Swerve Kinematics)

Autonomous Navigation, 3D SLAM, and Simulation stack for a 4-wheel independent swerve-drive rover in Gazebo Sim (Ignition Fortress) using **ROS 2 Humble** and **Nav2**.

---

## 📋 System Architecture & Overview

**MaRs-AutoNav** (`autonavtest`) provides an end-to-end autonomous navigation and mapping pipeline:
* **Rover Platform:** 4-wheel independent swerve-drive rover (`ares_nova`) featuring suspension, independent steering position controllers, and wheel drive velocity controllers.
* **Swerve Kinematics & Watchdog:** Custom Python node (`swerve_controller.py`) translating `/cmd_vel` into coordinated 4-wheel steering angles and drive speeds with $-90^\circ$ to $+90^\circ$ wheel angle optimization and a **0.5 s command timeout watchdog**.
* **Perception & Segmentation Pipeline:**
  * RGB-D camera point cloud generation (`rtabmap_util::point_cloud_xyz`).
  * Real-time terrain normal and slope segmentation (`rtabmap_util::obstacles_detection`) splitting the cloud into `/obstacles` (steep slopes, rocks, cliffs) and `/ground` (drivable flat terrain).
* **SLAM & Localization (RTAB-Map):**
  * **Mapping Mode:** Generates a 2D occupancy grid (`/rtabmap/map`) and 3D visual graph saved into `~/.ros/rtabmap.db`.
  * **Pure Localization Mode:** Preserves the database read-only (`localization:=true`), localizing against the saved map without modifying map geometry.
* **Sensor Fusion (EKF):** `robot_localization` EKF fusing wheel odometry and IMU into `/odometry/local` (`odom -> base_footprint`).
* **Nav2 Navigation Stack:**
  * **Global Planner:** NavFn A* finding optimal global paths on `/rtabmap/map`.
  * **Local Controller:** DWB Controller with tuned rotational limits and slowing factors for reliable in-place spot turns on complex terrain.
  * **Dynamic Costmaps:** 2-layer costmaps with `/obstacles` marking and `/ground` raytrace clearing to eliminate transient phantom obstacles.

---

## 🛠 Prerequisites & Dependencies

* **OS:** Ubuntu 22.04 LTS
* **ROS Distro:** [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)
* **Simulator:** Gazebo Sim (Ignition Gazebo Fortress 6.x) via `ros_gz`

### Install Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-rtabmap-launch \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-util \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-teleop-twist-keyboard \
  ros-humble-xacro
```

---

## 🚀 Build Instructions

1. **Clone the repository and switch to the `tharanesh` branch:**
   ```bash
   cd ~/MaRs-AutoNav
   git checkout tharanesh
   ```

2. **Build the workspace:**
   ```bash
   cd ~/MaRs-AutoNav/ros_ws
   colcon build --packages-select autonavtest
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.zsh   # or setup.bash
   ```

> **Network Isolation Note:** Always run `export ROS_DOMAIN_ID=42` in your terminals to isolate your ROS 2 nodes and clocks from foreign robot traffic on university/shared local area networks.

---

## 🎮 How to Run

To run the complete simulation, mapping, and navigation stack, open 3 terminal windows:

### Terminal 1: Gazebo Simulation & EKF
Spawns the rover at $(x=0, y=3)$ on the 3D terrain, starts controllers, the swerve brain, and the EKF:
```bash
export ROS_DOMAIN_ID=42
source ~/MaRs-AutoNav/ros_ws/install/setup.zsh
ros2 launch autonavtest launch_sim.launch.py
```

### Terminal 2: RTAB-Map SLAM & RViz2

* **Option A: Pure Localization Mode (Navigating on a saved map):**
  ```bash
  export ROS_DOMAIN_ID=42
  source ~/MaRs-AutoNav/ros_ws/install/setup.zsh
  ros2 launch autonavtest rtabmap.launch.py localization:=true
  ```

* **Option B: Mapping Mode (Building a new map):**
  ```bash
  export ROS_DOMAIN_ID=42
  source ~/MaRs-AutoNav/ros_ws/install/setup.zsh
  ros2 launch autonavtest rtabmap.launch.py localization:=false
  ```

### Terminal 3: Nav2 Autonomous Navigation
Starts the Nav2 lifecycle nodes (planner, controller, costmaps, smoother, behavior server):
```bash
export ROS_DOMAIN_ID=42
source ~/MaRs-AutoNav/ros_ws/install/setup.zsh
ros2 launch autonavtest nav2.launch.py
```

---

## 🎯 Navigating the Rover

1. **Single Goal Navigation:** In RViz, click the **2D Goal Pose** button on the top toolbar and click/drag a destination heading on the map. The rover will compute a path and autonomously drive to the goal.
2. **Multi-Waypoint Navigation:** Select **Nav2 Waypoint** mode in the RViz toolbar, click 2–3 sequential points across the map, and click **Start Navigation**.
3. **Manual Teleoperation (Optional):**
   ```bash
   export ROS_DOMAIN_ID=42
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

---

## 📁 Package Structure

```text
MaRs-AutoNav/
└── ros_ws/
    └── src/
        ├── CMakeLists.txt                # Package build rules
        ├── package.xml                   # Runtime & build dependencies
        ├── config/
        │   ├── ekf.yaml                  # robot_localization EKF config
        │   ├── nav2_params.yaml          # Nav2 planner, controller, and costmap configs
        │   └── ros2_controllers.yaml     # Steering and drive ros2_control configs
        ├── description/
        │   ├── chassis.xacro             # Chassis, camera, and sensor links
        │   ├── rover_properties.xacro    # CAD dimensions and swerve geometry
        │   ├── rover.urdf.xacro          # URDF assembler & Gazebo plugins
        │   ├── suspension.xacro          # Suspension arm definitions
        │   └── wheel.xacro               # Swerve steer & drive wheel macro
        ├── launch/
        │   ├── launch_sim.launch.py      # Gazebo world, rover spawn, bridge, EKF, swerve node
        │   ├── nav2.launch.py            # Nav2 navigation stack bringup
        │   └── rtabmap.launch.py         # RTAB-Map SLAM, pointcloud pipeline & RViz2
        ├── rviz/
        │   └── default.rviz              # Pre-configured RViz display profile
        ├── scripts/
        │   └── swerve_controller.py      # Swerve inverse kinematics node with 0.5s watchdog
        └── worlds/
            ├── world.sdf                 # Gazebo simulation world definition
            └── map2.stl                  # 3D terrain mesh
```

---

## 📡 Core Topics

| Topic | Type | Source | Purpose |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 / Teleop | Commanded rover velocities |
| `/steering_position_controller/commands` | `std_msgs/msg/Float64MultiArray` | `swerve_controller.py` | 4-wheel steering joint angles |
| `/drive_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | `swerve_controller.py` | 4-wheel drive motor velocities |
| `/odometry/local` | `nav_msgs/msg/Odometry` | `robot_localization` | Filtered EKF odometry (`odom -> base_footprint`) |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | `rtabmap_slam` | 2D global occupancy grid (`map -> odom`) |
| `/camera/cloud` | `sensor_msgs/msg/PointCloud2` | `point_cloud_xyz` | Raw 3D camera depth point cloud |
| `/obstacles` | `sensor_msgs/msg/PointCloud2` | `obstacles_detection` | Segmented lethal obstacle points for Nav2 marking |
| `/ground` | `sensor_msgs/msg/PointCloud2` | `obstacles_detection` | Segmented flat ground points for Nav2 clearing |
| `/plan` | `nav_msgs/msg/Path` | `planner_server` | Global NavFn A* path |
| `/local_plan` | `nav_msgs/msg/Path` | `controller_server` | DWB local trajectory |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 Costmap | 2D global costmap with inflation |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 Costmap | 2D rolling local costmap |
