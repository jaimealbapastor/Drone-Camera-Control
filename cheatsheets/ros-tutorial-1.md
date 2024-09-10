# Using turtlesim, ros2, and rqt

```cmd
docker run -it --env DISPLAY=192.168.64.48:0 -v "C:/Users/Jaime/Documents/GitHub/ws_ros2:/home/rosdev/ws_ros2" my_ros_image
```

```bash
sudo apt-get update
sudo apt-get install ros-humble-turtlesim 
# (already installed in env setup)

RUN apt-get update && apt-get install -y xauth
# (already installed in env setup)

sudo apt install ~nros-humble-rqt*

rqt
# or rqt --force-discover (no need for me)
```

## ROS2 Nodes

- `ros2 run <package_name> <executable_name>`
- ``

```bash
ros2 run turtlesim turtlesim_node
ros2 node list

ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

## Topics

```bash
rqt_graph 
```

```bash
ros2 topic list
ros2 topic list -t # List of topics with msg type
```

```bash
ros2 topic echo <topic_name> # Creates subscriber node to echo msg
ros2 topic info <topic_name> # Shows topic info (pubs/subs number)
```

```bash
ros2 topic list -t
ros2 interface show geometry_msgs/msg/Twist # Show type of msg

ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# 1Hz rate
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ros2 topic echo /turtle1/pose

ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'
ros2 topic pub /reference sensor_msgs/msg/TimeReference '{header: "auto", time_ref: "now", source: "dumy"}'
```

```bash
# rate of published data
ros2 topic hz /turtle1/pose
```

## Services

```bash
ros2 service list

ros2 service type <service_name>
ros2 service type /clear

ros2 service find <type_name>
ros2 service find std_srvs/srv/Empty

ros2 interface show <type_name>
ros2 interface show std_srvs/srv/Empty

ros2 service call <service_name> <service_type> <arguments>
ros2 service call /clear std_srvs/srv/Empty
```

## Parameters

```bash
ros2 param list

ros2 param get <node_name> <parameter_name>
ros2 param get /turtlesim background_g

ros2 param set <node_name> <parameter_name> <value>
ros2 param set /turtlesim background_r 150

ros2 param dump <node_name>
ros2 param dump /turtlesim > turtlesim.yaml

ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

## Actions

```bash
ros2 node info /turtlesim
ros2 node info /teleop_turtle

ros2 action list
ros2 action list -t

ros2 action info /turtle1/rotate_absolute

ros2 interface show turtlesim/action/RotateAbsolute

ros2 action send_goal <action_name> <action_type> <values>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

## Launch files

Can be written in python, XML or YAML (python recommended)

```bash
ros2 launch turtlesim multisim.launch.py
```

## Ros2 bag - recording topic data

```bash
mkdir bag_files
cd bag_files

ros2 topic list

ros2 bag record <topic_name>
ros2 bag record /turtle1/cmd_vel

# record multiple topics and give a name to the database
ros2 bag record -o my_register /turtle1/cmd_vel /turtle1/pose
```