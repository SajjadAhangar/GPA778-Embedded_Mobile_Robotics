import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from jerro_msgs.msg import MotorSpeed
import time
import math

class TwistToMotors(Node):
    def __init__(self):
        super().__init__('twist')
        # Initial constants
        self.l_robot = 0.175
        self.rayon = 0.035
        self.ticks_par_tour = 632.8
        self.previous_a = None
        self.previous_b = None
        self.previous_time_a = None
        self.previous_time_b = None

        # Encoder
        self.encoder_a_state = None
        self.encoder_b_state = None

        # /cmd_vel subscriber
        self.subscription_twister = self.create_subscription(Twist,'cmd_vel',self.listener_callback_twister,10)

        # Encoders         B is the right wheel, A is the left wheel.
        self.subscription_encoder_a = self.create_subscription(Int32,'encoder_a',self.listener_callback_encoder_a,10)
        self.subscription_encoder_b = self.create_subscription(Int32,'encoder_b',self.listener_callback_encoder_b,10)
        # motor/set_speed publisher
        self.publisher_ = self.create_publisher(MotorSpeed, '/motor/set_speed',10)
        print("Node was initialized successfully.")


    def listener_callback_twister(self, msg):
        v_robot = msg.linear.x
        w_robot = msg.angular.z
        #self.get_logger().info('I heard linear velocity x is equal to  "%s"' % v_robot)
        #self.get_logger().info('I heard angulijar velocity (w) is equal to: "%s"' % w_robot)

        #
        v_left_ms  = (v_robot - (w_robot * self.l_robot / 2.0))
        v_right_ms = (v_robot + (w_robot * self.l_robot / 2.0))
        
        v_left_ticks  = (v_left_ms * self.ticks_par_tour) / (2.0 * math.pi * self.rayon)
        v_right_ticks = (v_right_ms * self.ticks_par_tour) / (2.0 * math.pi * self.rayon)

        self.get_logger().info(f'I am going to publish: (A): {v_left_ticks:.2f} (ticks/s), {v_left_ms:.2f} (m/s) **** (B): {v_right_ticks:.2f} (ticks/s), {v_right_ms:.2f} (m/s)')

        # Publishing step
        motor_set = MotorSpeed()
        motor_set.motor_speed_a = v_left_ticks 
        motor_set.motor_speed_b = v_right_ticks
        self.publisher_.publish(motor_set)


    def listener_callback_encoder_a(self,msg):
        # Reading current time in nanoseconds
        current_time_a  = self.get_clock().now().nanoseconds / 1e9

        # Handle the first read
        if self.previous_a is None:
            self.previous_a = msg.data
            self.previous_time_a = current_time_a
            return

        dt_a = current_time_a - self.previous_time_a

        if dt_a > 0:
            current_a = msg.data
            speed_ticks = (current_a - self.previous_a ) / dt_a
            speed_ms    = ( speed_ticks * 2.0 * math.pi * self.rayon) / self.ticks_par_tour
            self.get_logger().info(f'Encoder A reads: "{speed_ms:.2f}" (m/s)--({speed_ticks:.2f} ticks/s)', throttle_duration_sec=1.0)
            self.previous_a = current_a
            self.previous_time_a = current_time_a

    def listener_callback_encoder_b(self,msg):
        # Reading current time in nanoseconds
        current_time_b  = self.get_clock().now().nanoseconds / 1e9

        # Handle the first read
        if self.previous_b is None:
            self.previous_b = msg.data
            self.previous_time_b = current_time_b
            return    

        dt_b = current_time_b - self.previous_time_b

        if dt_b > 0:
            current_b = msg.data
            speed_ticks = (current_b - self.previous_b ) / dt_b
            speed_ms    = ( speed_ticks * 2.0 * math.pi * self.rayon) / self.ticks_par_tour
            self.get_logger().info(f'Encoder B reads: "{speed_ms:.2f}" (m/s)--({speed_ticks:.2f} ticks/s)', throttle_duration_sec=1.0)
            self.previous_b = current_b
            self.previous_time_b = current_time_b



def main(args=None):
    rclpy.init(args=args)
    twist = TwistToMotors()
    rclpy.spin(twist)
    twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

