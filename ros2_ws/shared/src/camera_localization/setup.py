from setuptools import find_packages, setup
from os import path
from glob import glob

package_name = "camera_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            path.join("share", package_name, "launch"),
            glob(path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "sensor_msgs",
        "opencv-python",
        "cv_bridge",
    ],
    zip_safe=True,
    maintainer="Jaime Alba",
    maintainer_email="jaimitoalba@gmail.com",
    description="Camera localization package using epipolar geometry for drone positioning",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_publisher = camera_localization.camera_publisher:main",
            "camera_subscriber = camera_localization.camera_subscriber:main",
        ],
    },
)
