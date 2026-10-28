#!/usr/bin/env python3

"""
ROS2 Odometry node for calculating robot pose from wheel encoder ticks and broadcasting TF.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
from tf2_ros import TransformBroadcaster


class JerroOdometry(Node):
    def __init__(self):
        super().__init__('odometry')

        # Robot parameters
        self.axel_lenght = 0.1655
        self.radius = 0.035
        self.pulses_per_revolution = 1943

        # Pose state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder tick tracking
        self.current_a = 0
        self.current_b = 0
        self.previous_a = 0
        self.previous_b = 0

        # TF TransformStamped setup
        self.robot_pose = TransformStamped()
        self.robot_pose.header.frame_id = 'odom'
        self.robot_pose.child_frame_id = 'base_link'

        # Subscribers for Left (A) and Right (B) encoders
        self.subscription_encoder_a = self.create_subscription(Int32,'encoder_a',self.listener_callback_encoder_a,10)
        self.subscription_encoder_b = self.create_subscription(Int32,'encoder_b',self.listener_callback_encoder_b,10)
       
        # Transform Broadcaster and Timer (10 Hz update rate)
        self.br = TransformBroadcaster(self)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.update_odometry)

        self.get_logger().info('JerroOdometry node initialized successfully.')

    def listener_callback_encoder_a(self, msg: Int32) -> None:
        """Stores the latest tick count for encoder A (Left wheel)."""
        self.current_a = msg.data

    def listener_callback_encoder_b(self, msg: Int32) -> None:
        """Stores the latest tick count for encoder B (Right wheel)."""
        self.current_b = msg.data

    def update_odometry(self) -> None:
        """Periodic timer callback to compute kinematics and broadcast TF."""
        # Calculate tick differentials
        d_enc_a = self.current_a - self.previous_a
        d_enc_b = self.current_b - self.previous_b

        self.previous_a = self.current_a
        self.previous_b = self.current_b

        # Compute wheel linear displacements (meters)
        dis_l = (2.0 * math.pi * self.radius * d_enc_a) / self.pulses_per_revolution
        dis_r = (2.0 * math.pi * self.radius * d_enc_b) / self.pulses_per_revolution

        # Differential drive kinematics
        d_s = (dis_l + dis_r) / 2.0
        d_theta = (dis_r - dis_l) / self.axel_lenght

        # Update global pose
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)
        self.theta += d_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # self.get_logger().info(f'Pose -> x: {self.x:.3f} m, y: {self.y:.3f} m, theta: {math.degrees(self.theta):.2f}°')

        angles_in_radians = [0, 0, self.theta]
        rotation = R.from_euler('xyz', angles_in_radians, degrees=False)
        self.quat = rotation.as_quat()


        # The translation or orientation variable corresponds to the transformation values
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.transform.translation.x = float(self.x)
        self.robot_pose.transform.translation.y = float(self.y)
        self.robot_pose.transform.translation.z = 0.0
        self.robot_pose.transform.rotation.x = float(self.quat[0])
        self.robot_pose.transform.rotation.y = float(self.quat[1])
        self.robot_pose.transform.rotation.z = float(self.quat[2])
        self.robot_pose.transform.rotation.w = float(self.quat[3])
        self.br.sendTransform(self.robot_pose)
        
def main(args=None) -> None:
    rclpy.init(args=args)
    odometry_node = JerroOdometry()
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()