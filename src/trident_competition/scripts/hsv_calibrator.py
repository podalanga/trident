#!/usr/bin/env python3
"""
HSV Color Threshold Calibrator
Subscribes to camera topic and provides GUI sliders to tune HSV values.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').value
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        
        # Window setup
        self.window_name = "HSV Calibrator"
        cv2.namedWindow(self.window_name)
        
        # Create trackbars
        cv2.createTrackbar('H Min', self.window_name, 0, 179, self.nothing)
        cv2.createTrackbar('S Min', self.window_name, 0, 255, self.nothing)
        cv2.createTrackbar('V Min', self.window_name, 0, 255, self.nothing)
        cv2.createTrackbar('H Max', self.window_name, 179, 179, self.nothing)
        cv2.createTrackbar('S Max', self.window_name, 255, 255, self.nothing)
        cv2.createTrackbar('V Max', self.window_name, 255, 255, self.nothing)
        
        # Default values (Green-ish)
        cv2.setTrackbarPos('H Min', self.window_name, 40)
        cv2.setTrackbarPos('S Min', self.window_name, 50)
        cv2.setTrackbarPos('V Min', self.window_name, 50)
        cv2.setTrackbarPos('H Max', self.window_name, 80)
        cv2.setTrackbarPos('S Max', self.window_name, 255)
        cv2.setTrackbarPos('V Max', self.window_name, 255)
        
        self.get_logger().info(f"Subscribed to {image_topic}")
        self.get_logger().info("Adjust sliders to isolate the desired color.")
        self.get_logger().info("Press 'q' to quit.")

    def nothing(self, x):
        pass

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Get current trackbar positions
        h_min = cv2.getTrackbarPos('H Min', self.window_name)
        s_min = cv2.getTrackbarPos('S Min', self.window_name)
        v_min = cv2.getTrackbarPos('V Min', self.window_name)
        h_max = cv2.getTrackbarPos('H Max', self.window_name)
        s_max = cv2.getTrackbarPos('S Max', self.window_name)
        v_max = cv2.getTrackbarPos('V Max', self.window_name)

        # Create HSV bounds
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Convert to HSV and threshold
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Morphological operations to clean up noise (optional, matches main code)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Display
        # Stack images horizontally for comparison
        # Resize for better visibility if needed
        scale = 0.5
        h, w = cv_image.shape[:2]
        dim = (int(w * scale), int(h * scale))
        resized_orig = cv2.resize(cv_image, dim)
        resized_res = cv2.resize(res, dim)
        
        # Convert mask to BGR for stacking
        resized_mask = cv2.resize(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), dim)
        
        # Stack: Original | Mask | Result
        stacked = np.hstack((resized_orig, resized_mask, resized_res))
        
        cv2.imshow(self.window_name, stacked)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print(f"\nFinal Values:")
            print(f"Lower: ({h_min}, {s_min}, {v_min})")
            print(f"Upper: ({h_max}, {s_max}, {v_max})")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
