---
title: "Demo Instructions"
date: "2025-10-09"
summary: "Complete guide for running the rover demo including Lidar, Camera, Controllers, Mapping, and Navigation with troubleshooting tips."
writer: "Chaerin An, Selina Shrestha"
keywords: "ROS2, Demo, Mapping, Navigation, SLAM, Lidar"
pinned: true
---

## Start docker

Open a terminal and run:

```bash
xhost +local:docker # display setting :0
isaac_ros_docker_custom # custom command that starts isaac_ros-dev
```

Keep the docker terminal running and open another terminal:

```bash
isaac_ros_bash_custom
```

The above command starts a bash shell inside the docker container. To use ROS packages and commands, run:

```bash
source install/setup.bash
```

## Start ROS nodes

### Lidar

```bash
ros2 launch rplidar_ros view_rplidar_s3_launch.py
```

### Camera

```bash
ros2 launch rover_bringup realsense_camera.launch.py
```

### Controller Launch (wheel encoder + IMU + Sabertooth)

```bash
ros2 launch controller controller.launch.py
```

### Rover Description

```bash
ros2 launch rover_description display.launch.py
```

![TF Tree](/docs/demo-instructions/image.png)

→ Make sure you see all TFs except for the map

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

### Speech Integration with Navigation

```bash
ros2 run lidar_nav_bringup location_goal_sender
```

---

### Keyboard Controller

```bash
ros2 run controller keyboard_control
```

Use keys: [W] Forward, [S] Backward, [A] Left, [D] Right, [Q] Quit

---

### Speech Pipeline

Start Ollama Server docker container (Outside of docker)

```bash
jetson-containers run --name ollama dustynv/ollama:r35.4.1
```

Launch speech_processor (Inside of docker)

```bash
ros2 launch speech_processor speech_pipeline.launch.py
```

---

## Trouble shooting

If you see the error below

```bash
[slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1760129409.880 for reason 'discarding message because the queue is full'
```

then run static transforms

```bash
ros2 run lidar_nav_bringup static_tf_broadcasters
```

### Integration of Speech Pipeline with Nav2

Run location_goal_sender node in lidar_nav_bringup

```bash
ros2 run lidar_nav_bringup location_goal_sender
```

Subscribes to : matched_location

Publishes nav2 goal

To test, publish a landmark to matched_location:

```bash
ros2 topic pub --once matched_location std_msgs/msg/String "{data: 'coffee table'}"
```

You should see the goal coordinate in Nav2’s log

### Mic & Speaker fixes

When launching NoMachine, the mic and speaker devices were being forwarded to the laptop because of the default NoMachine settings.

#### Test

```bash
espeak "Testing"
arecord -f cd -D pulse -d 5 test_mic.wav
aplay test_mic.wav
```

If nothing recorded or played, run the commands below.

#### speaker

```bash
pactl list sinks short
pactl set-default-sink alsa_output.usb-Generic_USB2.0_Device_20121120222016-00.analog-stereo
pactl suspend-sink alsa_output.usb-Generic_USB2.0_Device_20121120222016-00.analog-stereo 0
```

#### microphone

```bash
pactl set-default-source alsa_input.usb-Clip-on_USB_microphone_UM02-00.mono-fallback
pactl suspend-source alsa_input.usb-Clip-on_USB_microphone_UM02-00.mono-fallback 0
pactl info | grep "Default Source"
```

push the volume button on the speaker and run test again

```bash
arecord -f cd -D pulse -d 5 test_mic.wav
aplay test_mic.wav
```

you also can try piper TTS.

```bash
printf "Hello Anteaters!" | /opt/piper/piper --model /models/piper/en_US-lessac-medium.onnx --output_file /tmp/tts.wav && aplay /tmp/tts.wav
```
