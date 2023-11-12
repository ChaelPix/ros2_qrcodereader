import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('qr_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.bridge = CvBridge()

        self.declare_parameter('image_path', 'qr.png')
        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.cv_image = cv2.imread(image_path)

    def timer_callback(self):
        msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
