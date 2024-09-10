#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

echo "# This is included by entrypoint.sh" >> ~/.bashrc

# setup ros2 environment
source /opt/ros/"$ROS_DISTRO"/setup.bash --
if [ -f '~/ros2_ws/install/setup.bash' ]; then
    source ~/ros2_ws/install/setup.bash --
fi

# add sourcing to .bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

echo "if [ -f '~/ros2_ws/install/setup.bash' ]; then" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "fi" >> ~/.bashrc

if ls /dev/video* 1> /dev/null 2>&1; then
    for device in /dev/video*; do
        sudo chmod 666 "$device"
    done
else
    echo "No /dev/video* devices found."
fi

echo "# End of entrypoint.sh" >> ~/.bashrc

exec "$@"