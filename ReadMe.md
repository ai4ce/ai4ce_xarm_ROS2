This is directly modified from [UFACTORY's official xArm documentation](https://github.com/xArm-Developer/xarm_ros2/tree/humble)

Here, I will just record some major modifications

## URDF/Visualization
- Add visualization capability for our 3D-printed GelSight mount
    - add `mount/gelsight_realsense_d405.urdf.xacro` to `xarm_description/urdf`
    - add `mount/visual/gelsight_realsense_d405.stl` to `xarm_description/meshes`
    - add relevant parameters to launch files.

## Teleoperation 
- Successfully implement joint-space and twist teleoperation.
    - add `JoyToServoPub_xArm6.cpp` to `xarm_moveit_servo/src`
    - modified relevant launch file and CMakeLists.txt.