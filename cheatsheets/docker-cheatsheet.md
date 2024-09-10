# Docker cheatsheet

## Main commands

`docker...`

- `image ls`, `images`
- `image pull`, `pull`
- `image rm`, `rmi`
- `image build`, `build -t [img_name] [dockerfile_path]`

- `container run`, `run -it`
- `container ls`, `ps`
- `container rm`, `rm -f`
- `container prune`
- `container start`, `start`
- `container stop`, `stop`
- `container exec`, `exec`

## Sharing files with container

Inside Dockerfile:
`COPY config/ /site_config/`

`docker run -it -v [abs_path_on_host]:[abs_path_in_container] my_image`

## ROS

`ros:humble` doesn't support GUI
`osrf/ros:humble-desktop-full` does

`docker run -it --user ros --network=host --ipc=host -v "$PWD\src:/host_src" my_image`

windows cmd: `ipconfig`  

`docker run -it --env DISPLAY=172.21.154.35:0 -v "C:/Users/Jaime/Documents/GitHub/ros2/ros2_ws:/home/rosdev/ros2_ws" my_ros_image`
