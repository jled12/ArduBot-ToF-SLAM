# Build and run

Target middleware: ROS 2 Humble, Python 3, ament/colcon. The PC reference is Ubuntu 22.04; the Raspberry project used Bullseye with its existing ROS installation. This bundle does not install ROS itself.

## Raspberry

Install the manufacturer's Arducam depth camera SDK that provides `ArducamTOFCamera.hpp` and `libArducamDepthCamera`. It remains an external dependency; no SDK binaries are redistributed.

From the repository root:

```bash
cd raspberry_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install
source install/setup.bash
export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ardubot_full_robot_bringup full_with_pc.launch.py
```

Set the UART device in `raspberry_ws/src/ardubot_serial_robot/config/serial_robot.yaml`; default `/dev/ttyS0`, 115200 baud. Rebuild after changing installed configuration. One serial node and one ToF process must own their respective devices.

## PC

Install the normal Humble navigation, SLAM, OctoMap and RViz packages through rosdep, with CycloneDDS (`ros-humble-rmw-cyclonedds-cpp`) on both hosts.

```bash
cd pc_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install
source install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list -t
ros2 launch ardubot_pc_bringup_pointcloud all_in_one.launch.py
```

Start the Raspberry first and confirm `/scan`, `/imu`, `/odom`, `/left_ticks`, `/right_ticks`, and `/point_cloud2` on the PC. Optional arguments: `use_nav2:=false`, `use_3d:=false`, `use_rviz:=false`.

The compact launch is newly assembled for this public bundle. It retains the October mapping choices and gives navigation a five-second startup delay; it is not the robot's original readiness-gated launcher. Runtime behavior must be checked on the real hardware.

## Chrony

PC master: `config/chrony/pc-master.conf`. Raspberry client: `config/chrony/raspberry-client.conf`.
Replace `ardubot-pc.local` in the client file with the real PC hostname, install each file as `/etc/chrony/chrony.conf`, and restart the chrony service. Check `chronyc tracking` and `chronyc sources -v` on both hosts before starting ROS. The PC can use Internet NTP and falls back to local stratum 8; the Raspberry uses the PC exclusively.

## Mapping and TF

Raspberry publishes only raw `odom → base_link`. PC odometry publishes only the correction `odom_bno → odom` and the encoder-based wheel joint states. PC URDF owns sensor/laser/IMU static frames. PC mapping owns the static `map → odom_bno` transform.

`use_scan_matching=false`, `do_loop_closing=false`, and `transform_publish_period=0.0` are retained from the stable October configuration. This records scans in the odometry frame; it does not perform active SLAM pose correction or loop closure. Do not add a second publisher for `map → odom_bno` without changing this configuration.

3D: `/point_cloud2 → octomap_server → /projected_map → /projected_map_cleared`. Resolution 0.025 m, projection z 0–0.06 m. Display ToF 3D and the projected grid using the included RViz configuration. The separate historical 2 Hz OctoMap-tree viewer is not required and is not bundled.

The projected FOV clearing estimates previously visible free space and is a visualization aid; it is not wired into the Nav2 safety costmaps.
