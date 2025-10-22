---
title: "How to Create a Docker Custom Image"
date: "2025-05-06"
summary: "Step-by-step guide to creating and managing custom Docker images for Isaac ROS development, including container snapshots and custom script configuration."
writer: "Selina Shrestha"
keywords: "Docker, Container Management, Custom Images"
---

1. Go into docker container and install required packages
2. Get container ID of current docker container using:

```bash
docker ps
```

Eg:

![image.png](/docs/docker-custom-image/image.png)

1. Save a new docker image as a snapshot of your docker container:

```bash
docker commit <container_id> isaac_ros_dev-aarch64-custom:<yyyymmdd>
```

After this, you should see your new image listed when you run

```bash
docker images
```

![image.png](/docs/docker-custom-image/image%201.png)

1. Edit BASE_NAME and TAG in run_dev_custom.sh (line 148 and 152)

```bash
vim /ssd/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/run_dev_custom.sh
```

![image.png](/docs/docker-custom-image/image%202.png)

1. Use aliases to start and execute docker

```bash
alias isaac_ros_docker_custom='cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev_custom.sh ${ISAAC_ROS_WS}' # start docker container
alias isaac_ros_bash_custom='docker exec -it --user root isaac_ros_dev-aarch64-custom-container /bin/bash' # open bash in the container
```
