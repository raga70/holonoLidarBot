import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
from math import radians
import math
import time

from std_msgs.msg import String 

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.make_square)

        self.turn_twist = Twist()
        self.turn_twist.linear.x = 0.0
        self.turn_twist.angular.z = radians(90)

        self.stop_twist = Twist()
        self.stop_twist.linear.x = 0.0
        self.stop_twist.angular.z = 0.0


        self.move_fwd_twist = Twist()
        self.move_fwd_twist.linear.x = 1.0

        self.get_logger().info("Twist publisher node has been started.")

    def turn_turtle_90_degrees(self):
        self.publisher_.publish(self.turn_twist)
        angle_to_turn = radians(90)
        angular_velocity = abs(self.turn_twist.angular.z)
        turn_time = angle_to_turn / angular_velocity
        lg_msg = f"{turn_time}"
        self.get_logger().info(lg_msg)
        self.publisher_.publish(self.stop_twist)

    def move_forward(self):
        self.publisher_.publish(self.move_fwd_twist)
        x_velocity = abs(self.move_fwd_twist.linear.x)
        distance_to_move = 10 
        move_time = distance_to_move / x_velocity
        lg_msg = f"{move_time}"
        self.get_logger().info(lg_msg)
        self.publisher_.publish(self.stop_twist)


    def make_square(self):
        for i in range(4):
            msg = f"iteration: {i}"
            self.get_logger().info(msg)
            self.turn_turtle_90_degrees()
            self.move_forward()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try: 
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()