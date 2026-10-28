#!/usr/bin/env python3

"""
ROS 2 node for converting Twist velocity commands to motor ticks and processing wheel encoders.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from jerro_msgs.msg import MotorSpeed

import time
import math

class TwistToMotors(Node):
    def __init__(self)-> None:
        super().__init__('twist')
        # Initial constants
        self.axel_lenght = 0.175
        self.radius = 0.035
        self.pulses_per_revolution = 632.8

        self.encoder_state = {
            'a': {'previous_ticks': None, 'previous_time': None},
            'b': {'previous_ticks': None, 'previous_time': None},
        }

        # /cmd_vel subscriber
        self.subscription_twister = self.create_subscription(Twist,'cmd_vel',self.listener_callback_twister,10)

        # Encoders: B is the right wheel, A is the left wheel.
        self.subscription_encoder_a = self.create_subscription(Int32, 'encoder_a', lambda msg: self.encoder_callback(msg, 'a'), 10)
        self.subscription_encoder_b = self.create_subscription(Int32, 'encoder_b', lambda msg: self.encoder_callback(msg, 'b'), 10)

        # motor/set_speed publisher
        self.publisher_ = self.create_publisher(MotorSpeed, '/motor/set_speed',10)

        print("TwistToMotors node has been initialized successfully.")


    def listener_callback_twister(self, msg) -> None:
        v_robot = msg.linear.x
        w_robot = msg.angular.z
        #self.get_logger().info('I heard linear velocity x is equal to  "%s"' % v_robot)
        #self.get_logger().info('I heard angulijar velocity (w) is equal to: "%s"' % w_robot)

        # Velocities of the left and right wheel (m/s)
        v_left_ms  = (v_robot - (w_robot * self.axel_lenght / 2.0))
        v_right_ms = (v_robot + (w_robot * self.axel_lenght / 2.0))

        # Convert linear velocity (m/s) to motor ticks/s
        conversion_factor = 2.0 * math.pi * self.radius
        v_left_ticks  = (v_left_ms  * self.pulses_per_revolution) / (conversion_factor)
        v_right_ticks = (v_right_ms * self.pulses_per_revolution) / (conversion_factor)

        self.get_logger().debug(
            f'Publishing speeds -> Left (A): {v_left_ticks:.2f} ticks/s ({v_left_ms:.2f} m/s) | '
            f'Right (B): {v_right_ticks:.2f} ticks/s ({v_right_ms:.2f} m/s)'
        )

        # Publishing 
        motor_set = MotorSpeed() 
        motor_set.motor_speed_a = v_left_ticks  # Left wheel
        motor_set.motor_speed_b = v_right_ticks # Right wheel
        self.publisher_.publish(motor_set)

    def encoder_callback(self, msg: Int32, wheel_id: str) -> None:
        current_time = self.get_clock().now().nanoseconds / 1e9
        state = self.encoder_state[wheel_id]

        # Handle the first read
        if state['previous_ticks'] is None:
            state['previous_ticks'] = msg.data
            state['previous_time'] = current_time
            return

        dt = current_time - state['previous_time']

        if dt > 0:
            current_ticks = msg.data
            speed_ticks = (current_ticks - state['previous_ticks']) / dt
            speed_ms = (speed_ticks * 2.0 * math.pi * self.radius) / self.pulses_per_revolution
            
            # Saving for the next iteration
            state['previous_ticks'] = current_ticks
            state['previous_time'] = current_time


def main(args=None):
    
    rclpy.init(args=args)
    twist_to_motors = TwistToMotors()
    
    try:
        rclpy.spin(twist_to_motors)
    except KeyboardInterrupt:
        pass

    finally:
        twist_to_motors.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()