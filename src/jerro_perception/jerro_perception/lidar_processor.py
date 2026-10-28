#!/usr/bin/env python3
"""
ROS 2 LiDAR processor node for Lab 2b: 3D point cloud assembly 
via coordinated servo sweeping and Action Server management.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import time
from jerro_msgs.action import StartScan
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_ros
from jerro_msgs.srv import SetServoPos
from laser_assembler.scan_assembler import ScanAssembler


class LidarProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_processor_node')

        # Callback group for concurrent action and service execution
        self._group = ReentrantCallbackGroup()

        # Servo calibration parameters (pulse width in microseconds)
        self.SWEEP_MIN_PULSE = 1300
        self.SWEEP_MAX_PULSE = 1900
        self.SWEEP_STEP_PULSE = 10
        self.CENTER_PULSE = 1500
        self.SETTLE_TIME = 0.3  # Time allowed for servo positioning
        self._servo_done = False

        # Store latest scan message container
        self.latest_scan = None

        # Publishers
        self.publisher_ = self.create_publisher(PointCloud2, '/points', 10)
        self.pc2_pub = self.create_publisher(PointCloud2, 'perception/lidar_pc2', 10)

        # Subscriber for incoming 2D scans (using sensor data QoS for Best Effort)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_profile_sensor_data, 
            callback_group=self._group
        )

        # TF2 Buffer and Listener for spatial coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_node = rclpy.create_node('lidar_traitement_tf')
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.assembler = ScanAssembler(self.tf_buffer, fixed_frame='base_link')

        # Servo Service Client
        self.servo_client = self.create_client(SetServoPos, '/set_servo_pos', callback_group=self._group)
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_servo_pos service...')

        # Action Server for coordinating scanning sequences
        self._action_server = ActionServer(
            self, 
            StartScan,
            '/perception/start_scan',
            execute_callback=self.execute_scan_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._group,
        )

        # Initialize servo to center position
        self.move_servo(self.CENTER_PULSE)
        
        self.get_logger().info('Lidar Processor Node initialized with StartScan Action Server.')

    def scan_callback(self, msg: LaserScan) -> None:
        """Continuously caches the latest laser scan message."""
        self.latest_scan = msg

    def goal_callback(self, goal_request) -> GoalResponse:
        """Validates incoming action goals."""
        self.get_logger().info('Received scan goal request.')
        if not self.servo_client.service_is_ready():
            self.get_logger().warn('Servo service not ready. Rejecting goal.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Handles scan cancellation requests."""
        self.get_logger().info('Scan cancellation requested.')
        return CancelResponse.ACCEPT

    def move_servo(self, pulse_us: int) -> None:
        """Asynchronously commands the servo motor position."""
        self._servo_done = False
        req = SetServoPos.Request()
        req.position = pulse_us
        future = self.servo_client.call_async(req)
        future.add_done_callback(self._on_servo_response)

    def _on_servo_response(self, future) -> None:
        """Callback triggered when servo position command completes."""
        self._servo_done = True

    def execute_scan_callback(self, goal_handle):
        """Executes the full servo sweep and point cloud assembly sequence."""
        self.get_logger().info('Executing 3D scan sweep...')
        self.assembler.clear()

        result = StartScan.Result()
        feedback = StartScan.Feedback()

        pulse_range = range(self.SWEEP_MIN_PULSE, self.SWEEP_MAX_PULSE + 1, self.SWEEP_STEP_PULSE)
        total_steps = len(pulse_range)

        for idx, servo_state in enumerate(pulse_range):
            # Check for cancellation request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.move_servo(self.CENTER_PULSE)
                result.success = False
                result.message = 'Scan canceled by client.'
                return result

            # Move servo to target angle and wait for settle time
            self.move_servo(servo_state)
            time.sleep(self.SETTLE_TIME)

            # Consume the latest cached scan into the assembler
            if self.latest_scan is not None:
                self.assembler.add_scan(self.latest_scan)
            else:
                self.get_logger().warn('No laser scan received yet during sweep step.')

            # Publish progress feedback
            feedback.progress = 100.0 * (idx + 1) / total_steps
            goal_handle.publish_feedback(feedback)

        # Assemble accumulated scans into a 3D PointCloud2 message
        cloud_msg = self.assembler.to_pointcloud2()
        self.pc2_pub.publish(cloud_msg)

        # Return servo to center and finalize goal
        self.move_servo(self.CENTER_PULSE)
        goal_handle.succeed()

        result.success = True
        result.message = '3D scan completed successfully.'
        self.point_cloud = cloud_msg  # Populate result field if defined in action message
        self.get_logger().info('3D scan completed and published.')
        return result
    
def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarProcessorNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()