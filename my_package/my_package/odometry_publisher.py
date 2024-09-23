import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import serial 
import math
from kinematics_mechanum_wheel import KinematicMechanumWheel
from math_utils import euler_to_quaternion

class OdometryPublishing(Node):

    def __init__(self) -> None:
        super().__init__('odometry_publishing')
        self.subscription = self.create_publisher(Odometry, 'turtle1/odom', 10)
        y_to_wheel = (15/100)
        x_to_wheel =  (15/100)
        radius = ((8/2)/100) 
        angle_from_wheels = np.pi/2
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.odom_timer_callback)
        self.wheel = KinematicMechanumWheel(y_to_wheel, x_to_wheel, radius, angle_from_wheels)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.last_time = self.get_clock().now()
        

    def odom_timer_callback(self):
        serial_read_back = self.serial_port.readline().decode('utf-8').strip()
        if serial_read_back:
            current_time = self.get_clock().now()
            delta_time = (current_time - self.last_time).now()
            split_data = serial_read_back.split(',')
            omega_1, omega_2, omega_3, omega_4 = map(float, split_data)
            robot_velocities = self.wheel.calculate_robot_velocities(np.array([omega_1, omega_2, omega_3, omega_4]))
            
            delta_x = robot_velocities[0] * delta_time
            delta_y = robot_velocities[1] * delta_time
            delta_theta = robot_velocities[2] * delta_time


            self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
            self.y += delta_x * math.cos(self.theta) + delta_y * math.sin(self.theta)
            self.theta += delta_theta
            odom = Odometry()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0

            orientation = euler_to_quaternion(0, 0, self.theta)