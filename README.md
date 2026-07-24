# Yashvant - Branch

## How to use code
Step 1:
    cd to your workspace
Step 2:
    colcon build --packages-select <package name>
step 3:
    source install/setup.bash
step 4:
    ros2 launch ares_nova_v2 launch_sim.launch.py world:=/path/to/your/world_file.sdf
step 5: 
    In a new terminal run teleop


## URDF preview
You shall preview the URDF using the preview.urdf file, in your VS CODE using URDF extension by smilerobotics
Here is a screenshot:
![URDF PREVIEW](/home/yashvant/ros2_ws/src/ares_nova_v2/screenshots/urdf_preview.png)