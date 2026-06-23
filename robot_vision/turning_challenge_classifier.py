import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class TurningChallengeClassifier(Node):
    def __init__(self):
        super().__init__('turning_challenge_classifier')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription( # TODO only send message over /vision/hsv_mask/black if it meets minimum pixel requirement
            Image,
            '/vision/hsv_mask/black',
            self.image_callback,
            10
        )
        
        
         # Minimum number of pixels in the mask to consider it valid
        self.declare_parameter('min_mask_pixels', 100)
        self.min_mask_pixels = self.get_parameter('min_mask_pixels').get_parameter_value().integer_value 
        
        self.declare_parameter('CONFIG_DEBUG', False)
        self.config_debug = self.get_parameter('CONFIG_DEBUG').get_parameter_value().bool_value
        self.debug_publisher = self.create_publisher(Image, '/vision/debug/turning_challenge_classifier', 10)

        self.direction_publisher = self.create_publisher(String, '/vision/turning_challenge/direction', 10)
        
    def image_callback(self, msg):
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        if not self.check_has_minimum_mask_pixels(mask):
            if self.config_debug:
                self.get_logger().info('Mask does not meet minimum pixel requirement')
            return
        
        
        arrow_contour = self.get_arrow_contour(mask)
        if arrow_contour is None:
            return
        
        # 4. Get the bounding box of the arrow
        x, y, w, h = cv2.boundingRect(arrow_contour)
        bbox_center_x = x + (w / 2)
        bbox_center_x_int = int(bbox_center_x)

        # Use the x position of the max-y pixel in the contour region.
        point_x, point_y = self.get_max_y_position(mask, arrow_contour)
        if point_x is None:
            if self.config_debug:
                self.get_logger().info('Could not determine max-y contour position')
            return

        direction = self.evaluate_and_publish(bbox_center_x_int, point_x)
        
        if self.config_debug:
            debug_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug_image, (point_x, point_y), 6, (0, 0, 255), -1)
            cv2.line(debug_image, (bbox_center_x_int, 0), (bbox_center_x_int, mask.shape[0] - 1), (255, 0, 0), 2)
            cv2.line(debug_image, (point_x, 0), (point_x, mask.shape[0] - 1), (255, 255, 0), 1)
            cv2.putText(debug_image, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_publisher.publish(debug_msg)
            self.get_logger().info(f'Published debug image with direction: {direction}')
        
    def check_has_minimum_mask_pixels(self, mask):
        if cv2.countNonZero(mask) < self.min_mask_pixels:
            return False
        return True
    
    def get_arrow_contour(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            if self.config_debug:
                self.get_logger().info('No arrow detected')
            return

            
        # Get the largest contour (assuming it's the arrow)
        arrow_contour = max(contours, key=cv2.contourArea)
        
        # simplify contour
        epsilon = 0.01 * cv2.arcLength(arrow_contour, True)
        simplified_contour = cv2.approxPolyDP(arrow_contour, epsilon, True)
        return simplified_contour

    def get_max_y_position(self, mask, contour):
        contour_mask = mask.copy()
        contour_mask[:] = 0
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        contour_pixels = cv2.bitwise_and(mask, contour_mask)

        points = cv2.findNonZero(contour_pixels)
        if points is None:
            return None, None

        max_y = int(points[:, 0, 1].min()) # turns out its reversed
        max_y_points = points[points[:, 0, 1] == max_y][:, 0, :]
        max_y_x = int(max_y_points[:, 0].mean())

        return max_y_x, max_y
    
    def evaluate_and_publish(self, bbox_center_x_int, point_x):
        direction = 'left' if point_x < bbox_center_x_int else 'right'
        direction_msg = String()
        direction_msg.data = direction
        self.direction_publisher.publish(direction_msg)
        return direction
        
    
        
def main(args=None):
    rclpy.init(args=args)
    node = TurningChallengeClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()