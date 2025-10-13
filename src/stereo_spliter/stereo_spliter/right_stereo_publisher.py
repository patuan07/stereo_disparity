import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RightStreamReceiver(Node):
    def __init__(self):
        super().__init__('right_stereo_publisher')
        self.pub_right = self.create_publisher(Image, '/right/image_rect', 10)
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture("http://172.21.176.1:8080/video", cv.CAP_FFMPEG)
        self.timer = self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No frame')
            return
        h, w, _ = frame.shape
        mid = w // 2
        right_img = frame[:, mid:, :]

        msg_right = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')

        now = self.get_clock().now().to_msg()
        msg_right.header.stamp = now
        msg_right.header.frame_id = "right_camera"

        self.pub_right.publish(msg_right)

def main():
    rclpy.init()
    node = RightStreamReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
