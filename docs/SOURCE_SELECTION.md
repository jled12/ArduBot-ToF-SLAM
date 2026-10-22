# Source selection and assembly changes

Selected references from the private project, consolidated to one version per component:

| Component | Selected source |
|---|---|
| Arduino | ardubot_V2_BNO055_V24_CMD_REJECT_348.ino |
| ToF | tof_pointcloud_median5x5_foreground.cpp, 18 October |
| UART | serial_robot_odom_tf_synchro.py; no duplicate joint-state publisher |
| BNO/encoder odometry | bno_encoder_odometry(2).py plus core.py from integration v3.3 |
| Heading correction and Nav2 | v3.3 integration sources/configuration |
| Robot model | wheels_clean_v0_0_8 URDF and three STL meshes |
| Projected grid | projected_map_clearing v0.3, resolution set to 0.025 m |
| Chrony | 7 October PC-master / Raspberry-client files, hostname generalized |

Packaging edits: UART/BNO YAML encoder defaults aligned to 348, unused EKF and demo packages removed, one copy of each required package retained, one TF owner per frame, sensor/laser static frames consolidated in the PC URDF, and public bringup/RViz resources assembled without absolute user paths. Existing motor PI and ToF algorithms are unchanged.

The serial node and BNO node both use the same raw odometry timeline; the visual wheel states are produced only by the BNO node. The full private archive is not part of this release's Git history.

This selection is a reproducible release candidate assembled from available sources, not a complete export of the exact currently installed software. Hardware, SDK/ROS build and camera-frame orientation still require a physical validation run.
