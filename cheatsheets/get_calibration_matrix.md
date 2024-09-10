# Calibration matrix with ROS2

- [Practical tutorial](https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555)
- [Official doc](https://docs.ros.org/en/rolling/p/camera_calibration/)

```bash
ros2 run camera_localization camera_publisher

ros2 run camera_calibration cameracalibrator --size 8x11 --square 0.012 \
  --ros-args -r image:=/sensor/camera/video0/frames
```
