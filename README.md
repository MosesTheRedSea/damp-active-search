# damp-active-search

> Developed at the Institute of Science Tokyo, Japan

DAMP-Active-Search (Disentangled Acoustic Multi-task Perception) is a robotics framework for active acoustic exploration and localization of occluded objects in indoor environments. It combines acoustic perception, belief map estimation, and information-driven navigation to enable a mobile robot to actively search for hidden targets using sound-based sensing.

> Docket Setup

Build the docker image

```bash
DOCKER_BUILDKIT=0 docker build -t foxy_acoustic -f docker/Dockerfile .
```


Run the container

```bash
docker run -it --net=host --name foxy_ros2 -v "$PWD/foxy_ws:/foxy_ws" foxy_acoustic
```

Start the container again

```bash
docker start -ai foxy_ros2
```

Build the workspace

```bash
source /opt/ros/foxy/setup.bash

