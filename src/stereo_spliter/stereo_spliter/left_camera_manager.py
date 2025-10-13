import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory
import yaml
import os


class CameraInfoPublisher(Node):
    def __init__(self, name, yaml_file, topic):
        super().__init__(name)

        pkg_share = get_package_share_directory('stereo_spliter')
        yaml_path = os.path.join(pkg_share, yaml_file)

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"YAML file not found: {yaml_path}")

        with open(yaml_path, 'r') as f:
            calib = yaml.safe_load(f)

        msg = CameraInfo()
        msg.width = calib.get('image_width', 0)
        msg.height = calib.get('image_height', 0)
        msg.k = calib.get('camera_matrix', {}).get('data', [0.0]*9)
        msg.d = calib.get('distortion_coefficients', {}).get('data', [])
        msg.r = calib.get('rectification_matrix', {}).get('data', [0.0]*9)
        msg.p = calib.get('projection_matrix', {}).get('data', [0.0]*12)
        msg.distortion_model = calib.get('distortion_model', '')

        self.msg = msg
        self.publisher = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(0.1, self.publish_info)

    def publish_info(self):
        self.msg.header.frame_id = self.get_name()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)


def main():
    rclpy.init()

    left = CameraInfoPublisher('left_camera', 'left.yaml', '/left/camera_info')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(left)
    executor.spin()

    left.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
