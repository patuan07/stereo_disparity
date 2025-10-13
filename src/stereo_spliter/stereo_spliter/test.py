import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StreamReceiver(Node):
    def __init__(self):
        super().__init__('stream_receiver')
        self.pub = self.create_publisher(Image, '/stereo/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv.VideoCapture("http://172.20.64.1:8080/video", cv.CAP_FFMPEG)
        self.timer = self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No frame')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = StreamReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
