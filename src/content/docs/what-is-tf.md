---
title: "What is TF?"
date: "2025-04-24"
summary: "Project setup, goals, and initial architecture notes."
---

### **What is a "Frame"?**

A **frame** is a coordinate system used to define the position and orientation of objects in space. Frames are essential for describing the position of sensors, robots, or objects relative to each other.

In ROS2, there can be multiple frames, and each frame can be used to describe different aspects of the robot or its environment.

For example:

- The **`/map`** frame is a **static** frame that represents the global reference for the environment. It typically doesn't move during runtime and is used to represent the global coordinates of the world.
- The **`/odom`** frame is centered on the robot's position and is used to track its **local movement**. This frame moves with the robot as it moves around, but it is subject to drift over time.
- The **`/camera_link`** frame is **attached** to the camera sensor mounted on the robot. Its position and orientation are fixed relative to the camera, and it might be located at a point like **(0.5, 0, 0)** relative to the **`/odom`** frame (if the camera is 0.5 meters ahead of the robot's center).

### **What Can TF Give Us?**

TF is a tool in ROS2 that allows us to keep track of the transformations between these frames. It helps us **transform data** (like sensor readings) from one frame to another.

For example:

- If a camera detects an object at **(0, 5, 1)** in the **`/camera_link`** frame (which is relative to the camera), we can use TF to **transform** this point into another frame, such as the **`/map`** frame.
- The transformation would be done through a sequence of frames, such as **`camera_link → odom → map`**. First, the point is transformed from **`/camera_link`** to **`/odom`**, and then from **`/odom`** to **`/map`**. This allows us to express the position of the object in the global coordinate system (**`/map`**).

![image.png](/docs/sample-tf.png)
