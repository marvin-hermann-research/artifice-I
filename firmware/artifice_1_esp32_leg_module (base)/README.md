# ARTIFICE I - Leg Controll Module

# Architecture Summary

## Purpose
Steuerungs-ESP Firmware für 2 Beine (3 DOF pro Bein) im [[ARTIFICE I - Projektübersicht]]

## Overview

```bash
firmware/
└── artifice_1_esp32_leg_module/
    ├── CMakeLists.txt
    ├── app-colcon.meta
    ├── include/
    │   ├── servo_driver.hpp
    │   ├── motion_controller.hpp
    │   ├── ros_interface.hpp
    │   └── diagnostics.hpp
    └── src/
        ├── app.cpp
        ├── servo_driver.cpp
        ├── motion_controller.cpp
        ├── ros_interface.cpp
        └── diagnostics.cpp
```

## Modules

- app: main enty point
- servo_driver: PWM, calibration, safety
- motion_controller: interpolation, velocity limits
- ros_interface: subscriptions / publishers, QoS, executor
- diagnostics: watchdog, health, error handling
## ROS Interfaces

- Sub: `/leg/<id>/cmd_joint_positions (int32MultiArray) [RELIABLE]`
- Sub: `/leg/<id>/cmd_joint_positions (int32MultiArray) [RELIABLE]`
- Pub: `/leg/<id>/log (std_msgs/String) — first char level [BEST]


One Subscriber per Leg
Publisher is only subscribed by the **brain esp module**
Publisher publishes String of whicht the first char determines the level eg 

D - Debug
L - Log
E - Error

# Safety

- limits enforced in servo_driver
- HW+SW watchdog
- emergency_stop topic