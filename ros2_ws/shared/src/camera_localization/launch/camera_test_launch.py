import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_devices = ["/dev/video0", "/dev/video2"]

    return LaunchDescription(
        [
            # === Publishers ===
            Node(
                package="camera_localization",
                executable="camera_publisher",
                name="camera_publisher_0",
                output="screen",
                # parameters=[{"device": camera_devices[0]}],
                arguments=["--device", camera_devices[0]],
            ),
            Node(
                package="camera_localization",
                executable="camera_publisher",
                name="camera_publisher_1",
                output="screen",
                # parameters=[{"device": camera_devices[1]}],
                arguments=["--device", camera_devices[1]],
            ),
            # === Subscribers ===
            Node(
                package="camera_localization",
                executable="camera_subscriber",
                name="camera_subscriber_0",
                output="screen",
                parameters=[{"device": camera_devices[0]}],
                arguments=["--device", camera_devices[0]],
            ),
            Node(
                package="camera_localization",
                executable="camera_subscriber",
                name="camera_subscriber_1",
                output="screen",
                # parameters=[{"device": camera_devices[1]}],
                arguments=["--device", camera_devices[1]],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
