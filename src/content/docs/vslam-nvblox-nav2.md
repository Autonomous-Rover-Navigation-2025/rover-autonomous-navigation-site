---
title: "Visual SLAM and 3D Mapping with NVBlox and RealSense Camera"
date: "2025-10-30"
summary: "Complete guide for mapping and localization using Visual SLAM, NVBlox, and Nav2 integration with RealSense camera for 3D voxel mapping and navigation."
writer: "Selina Shrestha, Chaerin An"
keywords: "VSLAM, NVBlox, Nav2, Visual SLAM, 3D Mapping, Localization"
---

## Mapping

1. Launch camera, VSLAM, Nvblox, Nav2

   ```bash
   ros2 launch rover_bringup bringup.launch.py
   ```

   ### What `bringup.launch.py` Does

   `bringup.launch.py` is the main launch file that starts all core perception and navigation components required for mapping and localization.

   It automatically includes and launches the following:

   - **RealSense Camera (`realsense_camera.launch.py`)**

     Initializes the Intel RealSense depth + RGB streams and publishes `/color`, `/depth`, and camera info topics.

   - **Visual SLAM (`visual_slam.launch.py`)**

     Runs Isaac ROS Visual SLAM to compute real-time pose (from camera + IMU).

     Allows map saving and loading through actions.

   - **NVBlox (`nvblox.launch.py`)**

     Builds a 3D voxel map using depth images and VSLAM poses.

     Produces meshes, TSDF/ESDF layers, and an obstacle costmap.

   - **Nav2 (`nav2.launch.py`)**

     Starts the full navigation stack—planner, controller, costmaps, and behavior tree.

     Uses NVBlox's ESDF layer to generate obstacle-aware navigation paths.

2. Visualize on RViz
3. Run keyboard control or Sabertooth motor control

   ```bash
   ros2 run controller keyboard_control
   ros2 run controller sabertooth_cmd_vel_bridge
   ```

4. Move the rover around with keyboard control to map the area.

   You will see something like this on RViz:

   ![RViz Visualization](/docs/vslam-nvblox-nav2/image.png)

5. Save map:

   ```bash
   ros2 action send_goal /visual_slam/save_map isaac_ros_visual_slam_interfaces/action/SaveMap "{map_url: './src/rover_bringup/maps'}" --feedback
   ```

   You will see something like this:

   ![Map Save Feedback](/docs/vslam-nvblox-nav2/image-1.png)

6. Stop the nodes that started from `bringup.launch.py`

## Load Map and Localize

1. Relaunch VSLAM

   ```bash
   ros2 launch rover_bringup bringup.launch.py
   ```

2. Load the previously saved map

   ```bash
   ros2 action send_goal /visual_slam/load_map_and_localize \
     isaac_ros_visual_slam_interfaces/action/LoadMapAndLocalize \
     "{map_url: './src/rover_bringup/maps',
       localize_near_point: {x: 0.0, y: 0.0, z: 0.0}}" \
     --feedback
   ```

   ![Map Load Feedback](/docs/vslam-nvblox-nav2/image-2.png)
