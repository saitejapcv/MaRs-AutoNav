# Mars Rover Ackermann Steering — ROS 2 Humble + Gazebo Classic

This package is a beginner-friendly simulation of a 4-wheel Mars-rover-style vehicle using Ackermann steering.

## What is included

- 4-wheel rover model
- Front-left and front-right steering joints
- Rear-left and rear-right driven wheels
- `ros2_control`
- `ackermann_steering_controller`
- Gazebo Classic simulation
- RViz visualization
- A small Python keyboard teleoperation node
- Launch files
- Controller YAML
- Xacro robot description

## Target platform

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- ROS 2 Python / rclpy

The package uses the ROS 2 Humble `ackermann_steering_controller`, which implements two steerable front wheels and two driven rear wheels.

---

# 1. Install ROS 2 Humble

If ROS 2 Humble is already installed, skip this section.

Follow the official ROS 2 Humble Ubuntu installation instructions:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

After installation:

```bash
source /opt/ros/humble/setup.bash
```

To source ROS automatically in every terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

# 2. Install required ROS packages

Run:

```bash
sudo apt update

sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions
```

Check the Ackermann controller:

```bash
ros2 pkg prefix ackermann_steering_controller
```

If that prints a path, it is installed.

Check Gazebo:

```bash
gazebo --version
```

---

# 3. Create the ROS 2 workspace

```bash
mkdir -p ~/mars_rover_ws/src
cd ~/mars_rover_ws/src
```

Extract/copy this package into `src`.

The final structure should look like:

```text
~/mars_rover_ws/
└── src/
    └── mars_rover_ackermann/
        ├── CMakeLists.txt
        ├── package.xml
        ├── README.md
        ├── config/
        │   └── controllers.yaml
        ├── launch/
        │   ├── simulation.launch.py
        │   └── rviz.launch.py
        ├── rviz/
        │   └── rover.rviz
        ├── scripts/
        │   └── keyboard_teleop.py
        └── urdf/
            └── mars_rover.urdf.xacro
```

---

# 4. Build

From the workspace:

```bash
cd ~/mars_rover_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Then:

```bash
source install/setup.bash
```

---

# 5. Start the simulation

Run:

```bash
ros2 launch mars_rover_ackermann simulation.launch.py
```

Gazebo should open with the rover.

In another terminal:

```bash
source ~/mars_rover_ws/install/setup.bash
```

Check controllers:

```bash
ros2 control list_controllers
```

You should see:

```text
joint_state_broadcaster
ackermann_steering_controller
```

---

# 6. Drive the rover

In another terminal:

```bash
source ~/mars_rover_ws/install/setup.bash
ros2 run mars_rover_ackermann keyboard_teleop.py
```

Keyboard:

```text
W = forward
S = reverse
A = steer left
D = steer right
X = stop
Q = quit
```

Use:

```text
W + A   forward-left
W + D   forward-right
S + A   reverse-left
S + D   reverse-right
X       stop
Q       quit
```

---

# 7. Test manually without the keyboard node

The controller accepts a stamped Twist reference.

Example forward command:

```bash
ros2 topic pub --once /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}"
```

Turn while moving:

```bash
ros2 topic pub --once /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.3}}}"
```

Stop:

```bash
ros2 topic pub --once /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

---

# 8. Useful checks

List topics:

```bash
ros2 topic list
```

Check joint states:

```bash
ros2 topic echo /joint_states
```

Check odometry:

```bash
ros2 topic echo /ackermann_steering_controller/odom
```

Check TF:

```bash
ros2 run tf2_tools view_frames
```

Check controller:

```bash
ros2 control list_controllers
```

---

# 9. RViz

After starting Gazebo, open another terminal:

```bash
source ~/mars_rover_ws/install/setup.bash
ros2 launch mars_rover_ackermann rviz.launch.py
```

---

# 10. If Gazebo opens but the rover does not move

First check:

```bash
ros2 control list_controllers
```

The Ackermann controller must be active.

Then:

```bash
ros2 topic info /ackermann_steering_controller/reference
```

Then send:

```bash
ros2 topic pub --once /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}"
```

---

# 11. How the rover works

The rover has:

- `front_left_steering_joint`
- `front_right_steering_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

The front steering joints receive steering position commands.

The rear wheel joints receive wheel velocity commands.

The Ackermann controller converts a body command:

```text
linear.x  = rover forward speed
angular.z = rover yaw rate
```

into individual wheel steering angles and wheel speeds.

This is more realistic for a car-like rover than a differential-drive model.

---

# 12. Next improvements for a Mars rover project

After this basic simulation works, add:

1. IMU
2. GPS
3. LiDAR
4. Camera
5. Wheel encoders
6. Nav2
7. SLAM
8. Obstacle avoidance
9. Autonomous waypoint navigation
10. Terrain/world models
11. Rover suspension
12. ROS 2 nodes for autonomous driving

