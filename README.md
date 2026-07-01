# damp-active-search

> Developed at the Institute of Science Tokyo, Japan
> 
Acoustic-Based Path Planning and Active Search for Occluded Object Localization with a Robot in Indoor Environments


## Docker Setup

Build the Docker image
```bash
DOCKER_BUILDKIT=0 docker build -t foxy_acoustic -f docker/Dockerfile .
```


Run the Container
```bash
docker run -it --net=host --name foxy_ros2 -v "$PWD/foxy_ws:/foxy_ws" foxy_acoustic
```

Build the workspace

```bash
source /opt/ros/foxy/setup.bash

