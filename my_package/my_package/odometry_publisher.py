import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import serial 
import math
from kinematics_mechanum_wheel import KinematicMechanumWheel
from math_utils import euler_to_quaternion
import tf2_ros

class OdometryPublishing(Node):

    def __init__(self) -> None:
        super().__init__('odometry_publishing')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        y_to_wheel = (15/100)
        x_to_wheel =  (15/100)
        radius = ((8/2)/100) 
        angle_from_wheels = np.pi/2
        timer_period = 0.1
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
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0

            orientation = euler_to_quaternion(0, 0, self.theta)
            odom.pose.pose.orientation = Quaternion(orientation)

            odom.twist.twist.linear.x = robot_velocities[0]
            odom.twist.twist.linear.y = robot_velocities[1]
            odom.twist.twist.angular.z = robot_velocities[2]
            # NOTE(Chris): estimates these should be updated with actual covariances
            odom.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                0, 0.01, 0, 0, 0, 0,
                                0, 0, 99999, 0, 0, 0,
                                0, 0, 0, 99999, 0, 0,
                                0, 0, 0, 0, 99999, 0,
                                0, 0, 0, 0, 0, 0.1]  

            odom.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 99999, 0, 0, 0,
                                 0, 0, 0, 99999, 0, 0,
                                 0, 0, 0, 0, 99999, 0,
                                 0, 0, 0, 0, 0, 0.1]  

            self.odom_publisher.publish(odom)
            self.publish_tf(current_time)
            self.last_time - current_time

        def publish_tf(self, current_time):
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            orientation =  euler_to_quaternion(0, 0, self.theta)
            t.transform.rotation.x = orientation[0] 
            t.transform.rotation.y = orientation[1] 
            t.transform.rotation.z = orientation[2] 
            t.transform.rotation.w = orientation[3] 

            self.tf_broadcaster.sendTransform(t)