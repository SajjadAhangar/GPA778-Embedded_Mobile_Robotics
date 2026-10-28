#!/usr/bin/env python3
"""
ROS 2 LiDAR processor node for converting 2D LaserScan data 
into Cartesian PointCloud2 messages for mobile robot perception.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 
from jerro_perception.laser_scan_to_array import laser_scan_to_array

class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_processor_node')

        # Publisher for 3D point cloud output
        self.publisher_ = self.create_publisher(PointCloud2, '/points', 10)

        # Subscriber for 2D laser scan input using sensor_data QoS (Best Effort)
        self.lidar_sub = self.create_subscription(LaserScan,"/scan",self.lidar_callback,qos_profile_sensor_data)

        self.get_logger().info('Lidar Processor Node initialized successfully.')

    def lidar_callback(self, msg: LaserScan) -> None:
        try:
            points = laser_scan_to_array(msg, remove_invalid_ranges=True)
            # Create the PointCloud2 message
            cloud_msg = point_cloud2.create_cloud_xyz32(msg.header, points)
            
            # Publishing point cloud
            self.publisher_.publish(cloud_msg)
        
        except Exception as e:
            self.get_logger().error(f'Failed to process LiDAR scan: {str(e)}')


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