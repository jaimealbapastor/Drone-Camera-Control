# Client libraries

## Colcon

```bash
sudo apt install python3-colcon-common-extensions 
# already installed for me

colcon build --symlink-install
colcon test

ros2 pkg create
```

## Workspace

```bash
rosdep update
# rosdep install -i --from-path src --rosdistro humble -y

colcon build
```

> Other useful arguments for `colcon build`:
>
> - `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
> - `--symlink-install` saves you from having to rebuild every time you tweak python scripts  
> - `--event-handlers console_direct+` shows console output while building (can otherwise be found in the log directory)
> - `--executor sequential` processes the packages one by one instead of using parallelism

## Create a package

```bash
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package

colcon build --packages-select my_package
```

# Simple pub sub package (Python)

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub

cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

# Build and run
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_pubsub

source install/setup.bash
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```

