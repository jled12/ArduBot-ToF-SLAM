# SLAM with Camera TOF

Minimal source distribution for the ArduBot differential-drive robot: Arduino motor control, Raspberry Pi Arducam ToF acquisition, and ROS 2 Humble mapping/navigation on a PC.

## Contents

| Folder | Purpose |
|---|---|
| `arduino/ardubot` | One Arduino sketch: encoders, motor PI control and BNO055 |
| `raspberry_ws/src` | ToF driver, serial node and domain bridge launch |
| `pc_ws/src` | BNO/encoder odometry, heading control, robot model, navigation and projected 3D occupancy grid |
| `config/chrony` | PC-master / Raspberry-client clock synchronization |
| `docs` | Installation, selected-source provenance and verification |

[Build and run](docs/BUILD_AND_RUN.md) · [Source selection](docs/SOURCE_SELECTION.md) · [Verification](docs/VERIFICATION.md)

This source-only release was assembled on 21 October 2026. It contains no development history, recordings, reports, calibration campaigns, private machine names, SDK binaries or built workspaces.

## Hardware and configuration

Raspberry Pi 4, Arducam ToF V2, BNO055, two geared motors with encoders, and the controller supported by the sketch's `microM` dependency. Reference geometry: 30 mm wheels, 78.4 mm track, 348 ticks/revolution. UART: 115200 baud; wheel commands are in m/s.

The exact firmware currently flashed on the author's robot could not be established from the recovered files. The included V24 sketch is a selected source reference, not a claim of hardware validation.

## Status

Core programs are retained from recovered sources. Bringup launch files and RViz configuration were simplified for this distribution, and conflicting encoder defaults were aligned to 348. This compact assembly has static and pure-Python checks; it has not been compiled with ROS/Arduino or tested on the physical robot here.

The retained October mapping profile uses odometry with scan matching and loop closure disabled. See the TF and mapping notes before enabling active SLAM correction.

## Dependencies and licensing

Arduino libraries and the Arducam SDK are installed separately. See [Arduino dependencies](arduino/DEPENDENCIES.md).

The project license is GPL-3.0, as selected for the source repository. Original per-package license declarations and third-party notices are preserved; external SDKs and libraries remain under their own terms. See [third-party notes](THIRD_PARTY.md).
