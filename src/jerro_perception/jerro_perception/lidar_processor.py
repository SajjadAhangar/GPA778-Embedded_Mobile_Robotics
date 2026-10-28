#!/usr/bin/env python3
"""
ROS 2 LiDAR processor node incorporating a dummy Action Server (StartScan)
for upcoming 3D servo-scanning capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from jerro_perception.laser_scan_to_array import laser_scan_to_array
from sensor_msgs.msg import PointCloud2 , LaserScan
from sensor_msgs_py import point_cloud2
import time

from rclpy.action import ActionServer
from jerro_msgs.action import StartScan

class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_processor_node')

        # Publisher for 3D point cloud output
        self.publisher_ = self.create_publisher(PointCloud2, '/points', 10)

        # Subscriber for 2D laser scan input using sensor_data QoS (Best Effort)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)

        # Action Server for coordinating scanning sequences
        self._action_server = ActionServer(self, StartScan, 'perception/start_scan', self.execute_callback_dummy)
        
        self.get_logger().info('Lidar Processor Node initialized with StartScan Action Server.')


    def lidar_callback(self, msg: LaserScan) -> None:
        """Converts incoming 2D LaserScan to a 3D PointCloud2 message and publishes it."""
        try:
            points = laser_scan_to_array(msg, remove_invalid_ranges=True)
            
            # Create the PointCloud2 message
            cloud_msg = point_cloud2.create_cloud_xyz32(msg.header, points)
            
            # Publishing point cloud
            self.publisher_.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process LiDAR scan: {str(e)}')


    def execute_callback_dummy(self, goal_handle):
        """Dummy execution loop simulating servo movement and feedback progression."""
        self.get_logger().info('Executing StartScan action goal...')
        
        number_of_steps = 10
        
        for i in range(number_of_steps):
            time.sleep(0.5)                      # the work (in Milestone 3: servo + scan)
            
            # Progress print
            self.get_logger().debug(f'Scan step {i + 1}/{number_of_steps} completed.')

            # Feedback
            feedback = StartScan.Feedback()
            feedback.progress = 100.0 * (i + 1) / number_of_steps
            
            # Send updates to the client
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()                   # mark goal as succeeded

        # Result
        result = StartScan.Result()
        result.success = True
        result.message = "Done"
        return result                           # result is sent to the client
    
def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
