import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StreamReceiver(Node):
    def __init__(self):
        super().__init__('split_stereo_publisher')
        self.pub_left  = self.create_publisher(Image, '/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/right/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture("http://172.21.176.1:8080/video", cv.CAP_FFMPEG)
        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No frame')
            return
        h, w, _ = frame.shape
        mid = w // 2
        left_img  = frame[:, :mid, :]
        right_img = frame[:, mid:, :]

        msg_left  = self.bridge.cv2_to_imgmsg(left_img,  encoding='bgr8')
        msg_right = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')

        now = self.get_clock().now().to_msg()
        msg_left.header.stamp = now
        msg_left.header.frame_id = "left_camera"
        msg_right.header.stamp = now
        msg_right.header.frame_id = "right_camera"

        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

def main():
    rclpy.init()
    node = StreamReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
