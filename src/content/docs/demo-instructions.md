---
title: "Demo Instructions"
date: "2025-10-09"
summary: "Complete guide for running the rover demo including Lidar, Camera, Controllers, Mapping, and Navigation with troubleshooting tips."
writer: "Chaerin An"
keywords: "ROS2, Demo, Mapping, Navigation, SLAM, Lidar"
---

## Lidar

```bash
ros2 launch rplidar_ros view_rplidar_s3_launch.py
```

## Camera

```bash
ros2 launch rover_bringup realsense_camera.launch.py
```

## Controller Launch (wheel encoder + IMU + Sabertooth)

```bash
ros2 launch controller controller.launch.py
```

## Rover Description

```bash
ros2 launch rover_description display.launch.py
```

![TF Tree](/docs/demo-instructions/image.png)

→ Make sure you see all TFs except for the map

## Point Cloud

```bash
ros2 launch rover_bringup point_cloud_xyz.launch.py
```

---

### Mapping

Launch with slam_toolbox and map_saver

```bash
ros2 launch lidar_nav_bringup nav2.launch.py slam:=True
```

If it launches successfully, you should be able to see a map with `/map` topic

![Map Topic](/docs/demo-instructions/image-1.png)

![Map Visualization](/docs/demo-instructions/image-2.png)

Save map:

```bash
cd src/lidar_nav_bringup/maps/
ros2 run nav2_map_server map_saver_cli -f my_map
```

---

### Navigation

Launch with AMCL and map_server

```bash
ros2 launch lidar_nav_bringup nav2.launch.py
```

---

### Keyboard Controller

```bash
ros2 run controller keyboard_control
```

Use keys: [W] Forward, [S] Backward, [A] Left, [D] Right, [Q] Quit

---

### Troubleshooting

If you see the error below

```bash
[slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1760129409.880 for reason 'discarding message because the queue is full'
```

then run static transforms

```bash
ros2 run lidar_nav_bringup static_tf_broadcasters
```
