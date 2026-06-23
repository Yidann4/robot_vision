import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('custom_image_publisher')
        self.publisher_ = self.create_publisher(Image, '/static_test/tuning_image', 10)
        self.timer = self.create_timer(1/10.0, self.timer_callback)  # 10fps
        self.bridge = CvBridge()
        
        
        self.declare_parameter(
            'image_path',
            'drc_test_image.jpg'
        )
        image_path = self.get_parameter('image_path').value
        
        self.image = cv2.imread(f'/ros2_ws/src/robot_vision/sample_images/{image_path}')
        if self.image is None:
            self.get_logger().error(f'Failed to load image from: {image_path}')
        else:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            self.get_logger().info(f'Loaded image from: {image_path}')
    
        if self.image is None:
            self.get_logger().error(f'Failed to load image from: {image_path}')
        else:
            self.get_logger().info(f'Loaded image from: {image_path}')
        
    def timer_callback(self):
        if self.image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.publisher_.publish(msg)
            # self.get_logger().info("Published image at time: " + str(msg.header.stamp))

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()