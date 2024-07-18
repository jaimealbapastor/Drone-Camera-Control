import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import argparse


class CameraPublisher(Node):
    def __init__(self, device="/dev/video0"):
        super().__init__(f"camera_publisher")

        device_id = device.split("/")[-1]
        self.publisher = self.create_publisher(
            Image, f"sensor/camera/{device_id}/frames", 5
        )

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"Publishing frames from {device}")

    def timer_callback(self) -> None:
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        else:
            self.get_logger().error("Failed to capture image")


def main(args=None):

    parser = argparse.ArgumentParser(description="Camera Publisher Node")
    parser.add_argument(
        "--device",
        type=str,
        default="/dev/video0",
        help="Camera device filename (default: /dev/video0)",
    )
    
    node_args, ros2_args = parser.parse_known_args()
    rclpy.init(args=ros2_args)
    
    cam_publisher = CameraPublisher(device=node_args.device)
    rclpy.spin(cam_publisher)
    cam_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
