---
title: "Controller Custom Package"
date: "2025-10-30"
summary: "Comprehensive documentation for the ROS2 Controller Package including keyboard control, Sabertooth motor bridge, wheel encoder, and wheel odometry publisher nodes."
writer: "Vaibhav Ashokbhai Gajera, Sahil Mukeshbhai Kakadiya"
keywords: "ROS2, Controller, Motor Control, Wheel Encoder, Odometry, Sabertooth"
---

![Controller Package Overview](/docs/controller-package/image.png)

![Controller Package Diagram 1](/docs/controller-package/image-1.png)

![Controller Package Diagram 2](/docs/controller-package/image-2.png)

![Controller Package Diagram 3](/docs/controller-package/image-3.png)

# **🛰️ ROS2 Keyboard Control Node**

---

### **1. Overview**

The **ROS2 Keyboard Control Node** provides manual control of a mobile robot or rover via keyboard inputs in real-time. It captures keypresses from the terminal and translates them into **velocity commands** using the **`geometry_msgs/msg/Twist`** message type.

These commands are then **published to the `/cmd_vel_nav` topic**, allowing the robot to move forward, backward, left, right, or stop based on the user input.

### **2. Key Features**

- Real-time manual control using keyboard (W, A, S, D, Q keys).
- Publish velocity data to the ROS2 topic `/cmd_vel_nav`.
- Implements smooth control by incrementing/decrementing velocities.
- Automatically stops the robot on exit (`Q` key).
- Provides console feedback for every keypress.

### 3. Node Purpose and Functionality

**Key Bindings**

| Key | Action        | Linear Velocity (x) | Angular Velocity (z) |
| --- | ------------- | ------------------- | -------------------- |
| W   | Move Forward  | +                   | 0                    |
| S   | Move Backward | -                   | 0                    |
| A   | Turn Left     | 0                   | +                    |
| D   | Turn Right    | 0                   | -                    |
| Q   | Stop and Exit | 0                   | 0                    |

The node captures each keypress, converts it into a **Twist** message, and publishes it. The `Twist` message structure:

```
linear.x = forward/backward speed
angular.z = left/right rotation
```

### **4. How It Works (Logical Flow)**

**Step-by-step Logic:**

1. The node initializes under the name `keyboard_control`.
2. A publisher is created for topic `/cmd_vel_nav`.
3. Inside a continuous loop:
   - The program waits for a keypress using `tty` and `termios` libraries.
   - Each keypress is mapped to a specific motion command.
   - The command is published as a `Twist` message to `/cmd_vel_nav`.
4. On pressing `Q`, the loop breaks:
   - Publishes a zero velocity message to stop the robot.
   - Node shuts down cleanly.

**Core Function Used:**

```python
def publish_twist(self, linear_x, angular_z):
    self.twist_msg_.linear.x = linear_x
    self.twist_msg_.angular.z = angular_z
    self.Serial_Com_pub.publish(self.twist_msg_)
```

This function converts numeric velocity values into a ROS2 `Twist` message and publishes it.

---

### **5. ROS2 Communication**

**Publisher:**

- **Topic:** `/cmd_vel_nav`
- **Message Type:** `geometry_msgs/msg/Twist`
- **Purpose:** Send motion commands to another node (Sabertooth bridge).

**No Subscriber:**

This node only sends data—it does not receive feedback.

---

### **6. How to Run**

**Terminal 1 – Launch ROS2 Core**

```bash
ros2 run controller keyboard_control
```

**Terminal Output Example:**

```
[INFO] [keyboard_control]: Use keys: [W] Forward, [S] Backward, [A] Left, [D] Right, [Q] Quit
You pressed W — moving forward with linear_velocity: 0.1, angular_velocity: 0.0
```

# **🛰️ ROS2 Sabertooth Motor Control Bridge (`sabertooth_cmd_vel_bridge.py`)**

### **1. Overview**

The **Sabertooth Bridge Node** acts as a translator between ROS2 velocity commands and the **Sabertooth 2x12 motor driver**.

It subscribes to the topic `/cmd_vel_nav` to receive `Twist` messages, computes individual left and right motor speeds, and sends appropriate serial commands to the Sabertooth motor controller via UART.

---

### **2. Key Features**

- Listens to velocity commands from `/cmd_vel_nav`.
- Computes left and right wheel velocities based on robot geometry.
- Sends motor commands over serial (`/dev/ttyTHS0`) to Sabertooth.
- Supports forward, reverse, and turning control.
- Uses checksum-based communication packets for reliability.
- Hardware input to Sabertooth S1 (Signal Input) is getting form Jetson Xavier Pin 8 (UART0_TX).

---

### **3. Node Purpose and Functionality**

This node bridges **ROS2** and the **Sabertooth motor driver**.

It interprets linear (`x`) and angular (`z`) velocities and converts them into **differential motor speeds**.

**Subscribed Topic:**

- `/cmd_vel_nav` (Message Type: `geometry_msgs/msg/Twist`)

---

### **4. How It Works (Logical Flow)**

**Step-by-step Logic:**

1. The node initializes with parameters:
   - `wheel_base` → Distance between the wheels (meters).
   - `max_speed` → Maximum linear wheel speed (m/s).
   - `port` → Serial device (default `/dev/ttyTHS0`).
   - `baud` → Baud rate (default 9600).
2. Establishes serial communication:

   ```python
   self.ser = serial.Serial(port, baud, timeout=0.1)
   ```

3. On receiving a `/cmd_vel_nav` message:

   - Extracts `linear.x` and `angular.z` values.
   - Calculates each wheel's speed:

     ```
     v_l = v + (w * L / 2)
     v_r = v - (w * L / 2)
     ```

   - Normalizes these speeds to a range [-127, 127].

4. Sends commands to Sabertooth using:

   ```python
   self.send_packet(address, command, value)
   ```

   Each command includes a checksum to ensure integrity.

---

### **5. Motor Control Logic**

| Motion Type | Condition                      | Motor Command                           |
| ----------- | ------------------------------ | --------------------------------------- |
| Forward     | linear.x > 0                   | Both motors forward                     |
| Backward    | linear.x < 0                   | Both motors reverse                     |
| Turn Left   | angular.z > 0                  | Left motor reverse, right motor forward |
| Turn Right  | angular.z < 0                  | Left motor forward, right motor reverse |
| Stop        | linear.x = 0 and angular.z = 0 | Send zero speed to both motors          |

---

### **6. Sabertooth Serial Packet Structure**

> ⚠️ Note: The Sabertooth DIP switches should be configured for Simplified Serial Mode (refer to the Sabertooth 2\*12V manual).

Each command sent to the Sabertooth motor driver follows this format:

```
[Address] [Command] [Value] [Checksum]
```

**Checksum Formula:**

```python
checksum = (address + command + value) & 0x7F
```

For example:

```
addr=128, cmd=0, val=60 → checksum=60
```

---

### **7. Address and Command Mapping**

| Motor | Address | Forward Cmd | Reverse Cmd |
| ----- | ------- | ----------- | ----------- |
| Left  | 129     | 0           | 1           |
| Right | 128     | 0           | 1           |

**Optional Ramp Commands:**

Used to make smoother speed transitions (commands 4 and 5).

---

### **8. ROS2 Communication**

**Subscriber:**

- **Topic:** `/cmd_vel_nav`
- **Message Type:** `geometry_msgs/msg/Twist`
- **Purpose:** Receive velocity commands.

**Serial Output:**

- **Port:** `/dev/ttyTHS0`
- **Baud Rate:** `9600`
- **Device:** Sabertooth 2x12 motor driver.

---

### **10. Conceptual Flow**

```
/cmd_vel_nav Topic (from Keyboard Control/ wheel odometry)
       ↓
sabertooth_cmd_vel_bridge Node
       ↓ (via UART Serial)
Sabertooth Motor Driver
       ↓
Motors (Left & Right)
```

---

### **11. How to Run**

**Individual Launch ROS2 node:**

```bash
ros2 run controller sabertooth_cmd_vel_bridge
```

**Entire Controller Package Launch file (Lunches the Sabretooth Node along with other nodes):**

```bash
ros2 launch controller controller.launch.py
```

**Example Output:**

```
[INFO] [sabertooth_cmd_vel_bridge]: Sabertooth bridge initialized and listening on /cmd_vel_nav
[INFO] [sabertooth_cmd_vel_bridge]: Sent: addr=128, cmd=0, val=65, chk=65
```

# **🛰️ ROS2 Controller Package** `wheel_encoder_node` Node **Documentation:**

## 🧭 Overview

This ROS 2 node (`wheel_encoder_node`) is designed to **read and interpret quadrature encoder signals** from four wheels on a robot, calculate their tick changes over time, and **publish those tick counts** on the ROS 2 topic `/encoder_ticks` as a `Float32MultiArray` message at 50 Hz.

It provides **low-level encoder ticks with direction feedback** necessary for formation of the wheel Odometry parameters like speed of each wheel.

## ⚙️ Node Overview

| Feature                      | Description                                            |
| ---------------------------- | ------------------------------------------------------ |
| **Node name**                | `wheel_encoder_node`                                   |
| **Purpose**                  | Generate the encoder raw data in terms of signed ticks |
| **Frequency**                | 50 Hz updates                                          |
| **ROS 2 Package Dependency** | `rclpy`, `Float32MultiArray`, `Jetson.GPIO`, `time`    |

## 🧾 Input / Output Specification

### 🟢 Input Actual Hardware GPIO PIN MAPPING

| Wheel       | A Pin | B Pin |
| ----------- | ----- | ----- |
| front_left  | 19    | 36    |
| front_right | 22    | 35    |
| rear_left   | 29    | 38    |
| rear_right  | 16    | 18    |

### 🔵 Output Topic

```
/encoder_ticks  Float32MultiArra [fl, fr, rl, rr]
```

## ⚙️ Quadrature Encoder Theory

### 🔹 What Is a Quadrature Encoder?

A **quadrature encoder** is a sensor that outputs two digital square waves : **Channel A** and **Channel B** shifted by 90 degrees in phase.

By monitoring the transitions of these signals, both the **direction** and **amount of rotation** can be determined.

| Encoder State | A   | B   | Binary | Meaning   |
| ------------- | --- | --- | ------ | --------- |
| State 0       | 0   | 0   | `00`   | Reference |
| State 1       | 0   | 1   | `01`   | +90°      |
| State 2       | 1   | 1   | `11`   | +180°     |
| State 3       | 1   | 0   | `10`   | +270°     |

When the encoder rotates **forward**, the states follow the sequence:

`00 → 01 → 11 → 10 → 00`

When it rotates **backward**, the sequence is reversed:

`00 → 10 → 11 → 01 → 00`

Each **transition** (state change) corresponds to a **quarter of a full encoder cycle**, allowing **4× resolution** compared to single-channel counting.

## 🧮 Decoding Logic (Quadrature Transition Table)

To interpret these transitions efficiently, the code uses a **4×4 transition lookup table**, where each row represents the **previous state** and each column represents the **current state**.

```python
STEP = [
#    0,  1,  2,  3   (curr)
    [ 0, +1, -1,  0],  # prev=0
    [-1,  0,  0, +1],  # prev=1
    [+1,  0,  0, -1],  # prev=2
    [ 0, -1, +1,  0],  # prev=3
]
```

### Interpretation:

- Each element gives the **tick delta**:

  +1 → forward step,

  –1 → reverse step,

  0 → no valid transition.

- Invalid jumps (e.g., both bits flip simultaneously) are ignored as **missed or noisy transitions**.

This method ensures **robust decoding** even if an intermediate state is missed due to limited sampling speed.

---

## 🧾 Execution Flow Summary

1. **Initialize ROS 2 node** and GPIO pins.
2. **Read initial encoder states** for all wheels.
3. **Start timer callback** at 50 Hz.
4. In each cycle:
   - Poll each wheel's A/B lines for 20 ms.
   - Decode transitions → get signed tick counts.
   - Publish `[fl, fr, rl, rr]` tick array.
5. On exit:
   - Clean up GPIO and safely shut down ROS 2.

### **How to Run**

**Individual Launch ROS2 node:**

```bash
ros2 run controller wheel_encoder_node
```

**Entire Controller Package Launch file (Lunches the** `wheel_encoder_node` **Node along with other nodes):**

```bash
ros2 launch controller controller.launch.py
```

# **🛰️ ROS2 Controller Package** `wheel_odom_publisher`Node **Documentation:**

## 🧭 Overview

> Node: wheel_odom_publisher

Purpose: Convert raw encoder tick data into real-world position (`x`, `y`, `θ`) and velocity (`vx`, `vy`, `ωz`) estimates, and publish these as standard ROS 2 odometry messages. Provide the Data to EKF node for Odom data Fusion.

---

## ⚙️ Node Overview

| Feature                      | Description                                                                |
| ---------------------------- | -------------------------------------------------------------------------- |
| **Node name**                | `wheel_odom_publisher`                                                     |
| **Purpose**                  | Integrate encoder ticks into position and velocity estimates               |
| **Frequency**                | Event-driven (whenever `/encoder_ticks` updates)                           |
| **ROS 2 Package Dependency** | `rclpy`, `tf2_ros`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `sensor_msgs` |
| Key Constants                | `TICKS_PER_ROTATION` → 620                                                 |

`WHEEL_DIAMETER`→ 0.1524 m
`CIRCUMFERENCE`→ π × D
`HALF_WHEEL_BASE`→ 0.105 m
`HALF_TRACK_WIDTH`→ 0.185 m
|

---

---

## 🧾 Input / Output Specification

### 🟢 Input Topic

```
/encoder_ticks  (std_msgs/Float32MultiArray)
```

- **Publisher:** `wheel_encoder_node`
- **Message format:**

  `[front_left, front_right, rear_left, rear_right]`

- **Units:** ticks since last publish
- **Direction:** Signed (+ forward, – backward)

### 🟠 Optional Input

```
/imu/data  (sensor_msgs/Imu)
```

_(future extension — currently yaw comes from `imu_yaw` placeholder)_

---

### 🔵 Output Topic

```
/wheel_odom  (nav_msgs/Odometry)
```

- **Fields:**
  - `pose.pose.position.x, y` → Robot position in meters
  - `pose.pose.orientation` → Quaternion from IMU yaw
  - `twist.twist.linear.x, y` → Linear velocities
  - `twist.twist.angular.z` → Angular velocity (from IMU)
- **Coordinate Frames:**
  - `frame_id`: `"odom"`
  - `child_frame_id`: `"base_link"`

### 🔵 TF Output

A **TransformStamped** frame is broadcast from `odom → base_link` using the same pose.

---

## 🧭 Execution Flow Summary

1. **Initialize ROS 2 node** `wheel_odom_publisher` and set up publishers/subscribers.
2. **Subscribe** to `/encoder_ticks` to receive tick deltas from all four wheels.
3. On each callback:
   - Compute **time difference (dt)** since last update.
   - Convert **tick counts → distances → velocities (vx, vy)**.
   - Integrate position to update **(x, y, θ)**.
4. **Publish** `/wheel_odom` (`nav_msgs/Odometry`) with pose and velocity data.
5. **Maintain TF broadcaster** for odom → base_link (if enabled).
6. **Shutdown gracefully** on exit, cleaning up node resources.

---

### **How to Run**

**Individual Launch ROS2 node:**

```bash
ros2 run controller wheel_odom_publisher
```

**Entire Controller Package Launch file (Lunches the** `wheel_odom_publisher`**Node along with other nodes):**

```bash
ros2 launch controller controller.launch.py
```
