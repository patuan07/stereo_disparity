import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PointPrinter(Node):
    def __init__(self):
        super().__init__('point_printer')
        self.sub = self.create_subscription(
            PointCloud2,
            '/points2',
            self.callback,
            100
        )

    def callback(self, msg):
        width = msg.width
        height = msg.height
        gen = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        count = 0
        for idx, p in enumerate(gen):
            y0 = idx // width
            x0 = idx % width
            x, y, z, rgb = p
            print(f"x0={x0}, y0={y0}, x={x:.3f}, y={y:.3f}, z={z:.3f}, rgb={int(rgb)}")
            count += 1
            if count >= 100:
                break

def main():
    rclpy.init()
    node = PointPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
