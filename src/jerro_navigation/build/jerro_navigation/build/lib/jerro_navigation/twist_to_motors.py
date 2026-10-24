import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from jerro_msgs.msg import MotorSpeed
import time

"""
class TwistToMotors(Node):
    def __init__(self):
        super().__init__('twist')
        # Initial constants
        self.l_robot = 0.175
        self.rayon = 0.035
        self.ticks_par_tour = 632.8
        self.previous_a = "Not set"
        self.previous_b = "Not set"
        self.previous_time_a = False
        self.previous_time_b = False

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
        #self.get_logger().info('I heard angular velocity (w) is equal to: "%s"' % w_robot)

        #
        v_left  = (v_robot - (w_robot * self.l_robot / 2.0) * self.ticks_par_tour) / (2.0 * 3.1415926 * self.rayon)
        v_right = (v_robot + (w_robot * self.l_robot / 2.0) * self.ticks_par_tour) / (2.0 * 3.1415926 * self.rayon)
        self.get_logger().info('I am going to publish "%s" to the left  wheel' % v_left)
        self.get_logger().info('I am going to publish "%s" to the right wheel' % v_right)

        # Publishing step
        motor_set = MotorSpeed()
        motor_set.motor_speed_a = v_left 
        motor_set.motor_speed_b = v_right
        self.publisher_.publish(motor_set)


    def listener_callback_encoder_a(self,msg):

        if self.previous_a == "Not set":
            self.previous_a = msg.data
            self.previous_time_a = self.get_clock().now().nanoseconds / 1e9 

        current_time_a  = self.get_clock().now().nanoseconds / 1e9
        dt_a = current_time_a - self.previous_time_a

        if dt_a > 1.0:
            current_a = msg.data
            speed_ticks = (current_a - self.previous_a ) / dt_a
            speed_ms    = ( speed_ticks * 2.0 * 3.1415926 * self.rayon) / self.ticks_par_tour
            self.get_logger().info('Encoder A reads: "%s" (m/s)' % speed_ms)
            self.previous_a = current_a
            self.previous_time_a = current_time_a

    def listener_callback_encoder_b(self,msg):

        if self.previous_b == "Not set":
            self.previous_b = msg.data
            self.previous_time_b = self.get_clock().now().nanoseconds / 1e9 

        current_time_b  = self.get_clock().now().nanoseconds / 1e9

        dt_b = current_time_b - self.previous_time_b

        if dt_b > 1.0:
            current_b = msg.data
            speed_ticks = (current_b - self.previous_b ) / dt_b
            speed_ms    = ( speed_ticks * 2.0 * 3.1415926 * self.rayon) / self.ticks_par_tour
            self.get_logger().info('Encoder B reads: "%s" (m/s)' % speed_ms)
            self.previous_b = current_b
            self.previous_time_b = current_time_b

"""

class TwistToMotors(Node):
    def __init__(self):
        super().__init__('twist')
        self.l = 0.175
        self.rayon = 0.035
        self.ticks_par_tour = 632.8
        self.subscription_twist = self.create_subscription(Twist,'cmd_vel',self.listener_callback_twist,10)
        self.subscription_encoder_a = self.create_subscription(Int32, 'encoder_a',self.listener_callback_encoder_a,10)
        self.subscription_encoder_b = self.create_subscription(Int32, 'encoder_b',self.listener_callback_encoder_b, 10)
        #self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(MotorSpeed, 'motor/set_speed', 10)

    def listener_callback_twist(self, msg):
        v_lineaire = msg.linear.x
        print(v_lineaire)
        w_angulaire = msg.angular.z
        v_convertit_droite = self.conversion_droite(v_lineaire, w_angulaire)
        print(v_convertit_droite)
        v_convertit_gauche = self.conversion_gauche( v_lineaire, w_angulaire)
        v_ticks_droite = self.conversion_en_ticks(v_convertit_droite)
        print(v_ticks_droite)
        v_ticks_gauche = self.conversion_en_ticks(v_convertit_gauche)

        output = MotorSpeed()
        output.motor_speed_b = v_ticks_droite
        output.motor_speed_a = v_ticks_gauche
        self.get_logger().info(f'Published: droite={v_ticks_droite}, gauche{v_ticks_gauche}')
        self.publisher_.publish(output)

    def listener_callback_encoder_a(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def listener_callback_encoder_b(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def conversion_gauche(self, v_lineaire, w_angulaire):
        v_gauche = v_lineaire - (w_angulaire * self.l) / 2
        return v_gauche

    def conversion_droite(self, v_lineaire, w_angulaire):
        v_droite = v_lineaire + (w_angulaire * self.l) / 2
        return v_droite

    def conversion_en_ticks(self, vitesse):
        ticks_par_seconde = (vitesse * self.ticks_par_tour) / (2 * 3.1415 * self.rayon)
        return ticks_par_seconde

def main(args=None):
    rclpy.init(args=args)
    twist = TwistToMotors()
    rclpy.spin(twist)
    twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

