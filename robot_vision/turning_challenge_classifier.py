import cv2
import numpy as np
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


        # Use densest mask-column location rather than centroid.
        densest_x, densest_y = self.get_densest_position(mask, arrow_contour)
        if densest_x is None:
            if self.config_debug:
                self.get_logger().info('Could not determine densest pixel position')
            return

        direction = self.evaluate_and_publish(bbox_center_x_int, densest_x)
        
        if self.config_debug:
            debug_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug_image, (densest_x, densest_y), 6, (0, 0, 255), -1)
            cv2.line(debug_image, (bbox_center_x_int, 0), (bbox_center_x_int, mask.shape[0] - 1), (255, 0, 0), 2)
            cv2.line(debug_image, (densest_x, 0), (densest_x, mask.shape[0] - 1), (255, 255, 0), 1)
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

    def get_densest_position(self, mask, contour):
        # 1. Create a mask just for this contour
        contour_mask = np.zeros_like(mask)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # 2. Extract the actual white pixels belonging to the contour
        contour_pixels = cv2.bitwise_and(mask, contour_mask)
        
        if cv2.countNonZero(contour_pixels) == 0:
            return None, None

        # 3. Apply Distance Transform
        # This calculates how far each white pixel is from the closest black background pixel
        dist_transform = cv2.distanceTransform(contour_pixels, cv2.DIST_L2, 5)
        
        # 4. Find the global maximum
        # The point furthest from any edge is the center of the biggest "blob"
        _, max_val, _, max_loc = cv2.minMaxLoc(dist_transform)
        
        # max_loc is returned as a tuple: (x, y)
        densest_x, densest_y = max_loc
        
        return int(densest_x), int(densest_y)
    
    def evaluate_and_publish(self, bbox_center_x_int, densest_x):
        direction = 'left' if densest_x < bbox_center_x_int else 'right'
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