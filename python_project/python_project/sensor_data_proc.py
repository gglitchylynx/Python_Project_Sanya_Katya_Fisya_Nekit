#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Camera viewer started")

    def cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"CV error: {e}")

def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()