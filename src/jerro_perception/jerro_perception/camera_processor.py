#!/usr/bin/env python3
"""
ROS2 Camera processor node for real-time edge detection and mobile robot perception.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_processor_node')

        # Parameters for Canny thresholds
        self.t_lower = 50
        self.t_upper = 150
        
        # Computer Vision Bridge
        self.bridge = CvBridge()

        # Subscribe using sensor_data QoS (Best Effort) to prevent buffer bloat on image streams
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, qos_profile_sensor_data)

        # Publish using Image interface
        self.publisher_ = self.create_publisher(Image, '/image_canny', 10)

        self.get_logger().info('Camera Processor Node initialized successfully.')

    def image_callback(self, msg) -> None:
        try:
            # Convert ROS Image message to OpenCV BGR8 format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Apply Canny edge detection
            img_canny = cv2.Canny(cv_image, self.t_lower, self.t_upper)

            # Convert back to ROS Image message (mono8 for single-channel edge maps)
            ros_image = self.bridge.cv2_to_imgmsg(img_canny, encoding="mono8")
            
            # Publish processed edge image
            self.publisher_.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f'Failed to process incoming image frame: {str(e)}')

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()