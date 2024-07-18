import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse


class CameraSubscriber(Node):
    def __init__(self, device="/dev/video0"):
        super().__init__(f"camera_subscriber")

        device_id = device.split("/")[-1]
        self.subscription = self.create_subscription(
            Image, f"sensor/camera/{device_id}/frames", self.listener_callback, 50
        )

        self.bridge = CvBridge()
        self.window_name = f"Camera {device}"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.get_logger().info(f"Receiving video frames from {device}")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)


def main(args=None):
    
    parser = argparse.ArgumentParser(description="Camera Subscriber Node")
    parser.add_argument(
        "--device",
        type=str,
        default="/dev/video0",
        help="Camera device filename (default: /dev/video0)",
    )
    
    node_args, ros2_args = parser.parse_known_args()
    rclpy.init(args=ros2_args)

    cam_subscriber = CameraSubscriber(device=node_args.device)
    rclpy.spin(cam_subscriber)
    cam_subscriber.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
