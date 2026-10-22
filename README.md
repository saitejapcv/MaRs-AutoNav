## How to use code
Step 1:
    cd to your workspace
Step 2:
    colcon build --packages-select autonavtest
step 3:
    source install/setup.bash
step 4:
    ros2 launch autonavtest launch_sim.launch.py 
step 5: 
    In a new terminal run teleop

This launch starts Gazebo, RGB-D synchronization, RTAB-Map and RTAB-Map Viz.
Do not start another RTAB-Map launch that subscribes directly to
`/depth_camera/image_raw/image`, `/depth_camera/image_raw/depth_image` and
`/depth_camera/image_raw/camera_info`: those independently bridged streams can
have different header timestamps. The supplied launch instead sends the
synchronized `/rgbd_image` stream to both RTAB-Map nodes.

To verify the camera path while the simulator is running:

    ros2 topic hz /depth_camera/image_raw/image
    ros2 topic hz /depth_camera/image_raw/depth_image
    ros2 topic hz /rgbd_image


## URDF preview
You shall preview the URDF using the preview.urdf file, in your VS CODE using URDF extension by smilerobotics
Here is a screenshot:
![URDF PREVIEW](src/autonavtest/screenshots/urdf_preview.png)
I f couldn;t view from here please view in screenshots folder.
