# Simple mmWave Drone Control
Simulate mmWave radar based drone control in Gazebo with ROS2 and PX4

Derived from: https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo

### Prerequisites
Tested with:
- Ubuntu 20.04.3 LTS
- ROS2 Foxy
- Gazebo 11.9.0
- px4_ros_com 29th Nov
- PX4 Autopilot v1.12.3


### Launch all
https://docs.px4.io/master/en/ros/ros2_offboard_control.html
https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp

0. If offboard_control.cpp or other files have been edited, re-run ```install.sh``` script (add new files to script and CMakeLists.txt):
   ```sh
   cd ~/mmWave_ROS2_PX4_Gazebo/
   ( chmod +x ./install.sh )
   ./install.sh
   ```
    If same PX4 and px4_ros_com_ros2 roots:
    ```
    ./install.sh ~/PX4-Autopilot/ ~/px4_ros_com_ros2/
    ```
1. Launch PX4 SITL:
   ```sh
    cd ~/PX4-Autopilot/ 
    make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without Gazebo GUI:
   ```sh
    HEADLESS=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without drone following:
   ```sh
    PX4_NO_FOLLOW_MODE=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   After PX4 SITL fully launched, might need to manually start microRTPS client in same terminal:
   ```sh
    micrortps_client start -t UDP
   ```
   Will fail and return -1 if already running.
2. Open QGroundControl   
3. In a new terminal start microRTPS agent and offboard control:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   micrortps_agent start -t UDP & ros2 run px4_ros_com offboard_control 
   ```
4. In another terminal, start the velocity vector advertiser, lidar to mmwave converter, and 3d to 2d projection nodes:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 launch ~/mmWave_ROS2_PX4_Gazebo/launch/simulate_pointcloud_control_launch.py 
   ```
5. Simulated drone in Gazebo should arm and takeoff. May need to restart ```vel_ctrl_vec_pub``` and ```offboard_control``` ros2 runs.

6. Visualize simulated data in rviz2:
   ```sh 
   rviz2 ~/mmWave_ROS2_PX4_Gazebo/3d_and_2d_pointcloud_rgb.rviz 
   ```


### MISC
1. Trajectory setpoint message:
   https://github.com/PX4/px4_msgs/blob/ros2/msg/TrajectorySetpoint.msg
2. Disabled param:
   pxh> param set NAV_RCL_ACT 0

   NAV_RCL_ACT: curr: 2 -> new: 0
3. Local positioning?
   https://github.com/PX4/px4_msgs/blob/ros2/msg/VehicleLocalPositionSetpoint.msg
   
4. Add any new ROS2 files to ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt

5. Check if drone armed? https://github.com/PX4/px4_msgs/blob/ros2/msg/ActuatorArmed.msg

6. libignition-common3 error (after software update?) - Copy existing file and rename to match missing file
7. If gazebo does not open, try running ```gazebo --verbose``` to troubleshoot. ```killall gzserver``` should kill any gazebo instances. Restart PC if all else fails.
8. inlude both iris.sdf and iris.sdf.jinja?
9. Implemented laser scanner with Gazebo and ROS2 https://github.com/chapulina/dolly
10. Make custom sensor plugin http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
11. In ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt add ```sensor_msgs``` under ```ament_target_dependencies```
12. After running ```./build_ros2_workspace``` restart all affected executables (micrortps_agent, offboard_control, vel_vec_ctrl_pub). Gazebo PX4 SITL can be left running.
13. iris.sdf (or other models) can be edited to include sensors, like 2D lidar.
14. Display simulated camera feed either with rviz2 or
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run image_tools showimage image:=/cable_camera/image_raw
   ```
14. Add new worlds/models to ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake (Oscar's worlds/models from https://gitlab.drones4energy.dk/obs/Drones4Energy_SDU_Only_code/-/tree/iROS2021/Tools/simulationAssets)
15. See local packages, and msgs, with: ```ros2 interface packages``` and e.g. ```ros2 interface package px4_msgs```
16. Camera intrinsic parameters for setting a custom perspective projection matrix (cannot be used with WideAngleCamera since this class uses image stitching from 6 different cameras for achieving a wide field of view). The focal lengths can be computed using focal_length_in_pixels = (image_width_in_pixels * 0.5) / tan(field_of_view_in_degrees * 0.5 * PI/180) (http://sdformat.org/spec?ver=1.7&elem=sensor#lens_intrinsics)
17. Drone spawn coordinates set in ~/PX4-Autopilot/Tools/sitl_run.sh ?
18. ```*** No rule to make target '/opt/ros/foxy/lib/libfastrtps.so.2.0.2', needed by 'libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so'.  Stop.```
Fixed by renaming closest libfastrtps.so.x.y.z to libfastrtps.so.2.0.2.
19. Dependency errors with PX4, like ```ninja: error: '/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.6.1', needed by 'libmav_msgs.so', missing and no known rule to make it``` may be solved by a PX4 reinstall (remember worlds, models, cmake files etc. must be also be reinstalled into new PX4).
20. If drone enters failsafe when starting offboard_control, ```param set COM_RCL_EXCEPT 4``` in the PX4 console may solve this. Else, try manually publish few setpoints to fmu/manual_control_setpoint/in and then start offboard mode.
21. Showing videos in readme: Just drag and drop your image/video from your local pc to github readme in editable mode.
22. If gradle not working, might have to downgrade Java (JDK) to 11: https://askubuntu.com/questions/1133216/downgrading-java-11-to-java-8




### TODO
0. :green_circle: Install tools 
1. :green_circle: Figure out how to control drone via offboard_control.cpp 
2. :green_circle: Make ROS2 advertiser that generates control input for offboard_control.cpp for more advanced control
3. :green_circle: Figure out how to use simulated depth sensors
4. :green_circle: Implement depth data into ROS2 advertiser for even more advanced control
5. :green_circle: Control drone towards overhead cable

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-06-08_15-17-35.png?raw=true)

7. :yellow_circle: More tightly integrate with PX4 to optimize control based on e.g. drone state
   - get pose of drone to mitigate sideways motion when rotated around x or y.
   - use GPS positioning to counteract drift
9. :green_circle: Use drone mounted simulated camera to get images of overhead cable 
10. :green_circle: Visualize depth data in camera feed

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-07-05_20-52-58.png?raw=true)

12. :green_circle: Investigate occasional drone control loss
13. :green_circle: Make module that turns 2d lidar data into noisy pointcloud to prepare for mmwave integration
14. :yellow_circle: Tracking of points in pointcloud (kalman?)
15. :yellow_circle: Implement cable detection AI to filter depth data and align drone yaw wrt. cable

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-07-06_09-54-53.png?raw=true)

https://user-images.githubusercontent.com/76950970/142616665-0cb08003-9355-4eed-a658-7d900c9f66fb.mp4



