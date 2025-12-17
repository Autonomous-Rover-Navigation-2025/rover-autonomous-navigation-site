---
title: "Project Overview"
date: "2025-10-26"
summary: "Complete system architecture overview of the Rover Autonomous Navigation system, integrating hardware sensors, ROS 2 perception, navigation, control, and speech command processing."
writer: "Chaerin An"
keywords: "System Architecture, ROS2, Navigation, Perception, Hardware Integration"
pinned: true
---

## 🧭 Rover Autonomous Navigation – System Overview

The Rover Autonomous Navigation system integrates multiple hardware sensors, embedded software modules, and AI-driven perception and control pipelines. The overall architecture is divided into 5 layers: **Hardware**, **ROS 2 Perception**, **Navigation**, **Control**, and **Speech Command Processing**.

![System Architecture](/docs/project-overview/architecture.png)

---

### **1. Hardware Layer**

This layer represents the physical components of the rover that interact directly with the environment.

- **NVIDIA Jetson Xavier NX** serves as the main onboard computer running ROS 2 nodes and real-time processing.
- **RPLIDAR A2M7** provides 2-D laser scans (`/scan`) for mapping and obstacle detection.
- **Intel RealSense D435i** supplies depth images and IMU data for perception and localization.
- **Wheel Encoders** generate odometry ticks (`/wheel_odom`) used to estimate linear and angular velocity.
- **Onboard IMU** measures acceleration and angular rate for motion estimation.
- **Sabertooth Motor Driver** controls two DC motors for actuation.
- **Microphone** captures user voice input for speech commands.

---

### **2. ROS 2 Perception Layer**

This layer fuses raw sensor data to produce reliable robot state estimation.

- **Madgwick Filter** processes raw IMU data from the RealSense to generate filtered orientation (`/imu/data`).
- **Wheel Odom Node** converts encoder ticks into odometry (`/odom_raw`).
- **robot_localization EKF** combines wheel odometry and IMU orientation to output a fused pose (`/odom`), which is the robot's local frame of reference.
- **SLAM Toolbox** builds a map during exploration, while **AMCL** localizes the robot on a pre-built map.
- **TF Tree** maintains coordinate transformations (`odom → base_link → sensors`).

This multi-sensor fusion ensures that both local odometry and global localization remain consistent and drift-free.

---

### **3. Navigation & Behavior Layer**

This layer handles global path planning, local obstacle avoidance, and task orchestration.

- **Nav2 Costmaps** aggregate LiDAR and map data to represent the traversable area.
- **Global Planner** computes the optimal path to the target destination.
- **Local Planner (DWB/TEB)** generates real-time velocity commands around obstacles.
- **Behavior Tree (BT)** coordinates mission-level decisions such as escorting, patrolling, or charging, based on incoming goals or user commands.

---

### **4. Control & Actuation Layer**

This layer executes the motion commands produced by the navigation stack.

- Velocity commands (`/cmd_vel`) are transmitted to the **Sabertooth Driver**, which actuates the motors to achieve the desired movement.
- Feedback loops ensure smooth and stable motion.

---

### **5. Speech Command Processing Layer**

This layer converts natural speech into actionable navigation goals.

- The **Microphone** captures user speech.
- The **Wake Word Detector** activates the pipeline when a trigger phrase (e.g., "Hey Rover") is detected.
- The **Intention Classifier** interprets the semantic intent behind the utterance.
- The **Mission and Destination Extractor** identifies the specific mission type (escort, patrol, emergency) and destination coordinates.
- Extracted commands are sent to the **Voice App** interface and ultimately to the **Behavior Tree**, which initiates the corresponding navigation routine.

This pipeline enables natural, hands-free human-robot interaction, connecting spoken intent directly to autonomous navigation.

---

### **Overall Flow**

1. **Sensors** collect environmental and motion data.
2. **Perception (EKF + SLAM)** fuses these inputs to maintain localization.
3. **Navigation & BT** plan paths and execute mission logic.
4. **Control** actuates the motors accordingly.
5. **Speech Pipeline** allows users to control the rover verbally, translating language into navigation actions.

---
