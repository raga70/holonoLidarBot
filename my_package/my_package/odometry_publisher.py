from re import split
from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import serial 
import math
from kinematics_mechanum_wheel import KinematicMechanumWheel
from math_utils import euler_to_quaternion
from variance_calculations import var_gearbox_backlash, var_resolution
from general_utils import fill_odometry_message, convert_serial_data_to_angular_velocities
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
        self.radius = ((8/2)/100) 
        angle_from_wheels = np.pi/2
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.odom_timer_callback)
        self.wheel = KinematicMechanumWheel(y_to_wheel, x_to_wheel, self.radius, angle_from_wheels)
        self.serial_port = serial.Serial('/dev/Serial0', 9600, timeout=1)
        self.last_time = self.get_clock().now()
        self.var_encoding = var_resolution(self.radius, 1440)

    def tf_publish(self, current_time):
        print('publishing tf')
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

    def odom_timer_callback(self):
        serial_read_back = self.serial_port.readline().strip()
        if serial_read_back:
            current_time = self.get_clock().now()
            delta_time = (current_time - self.last_time).nanoseconds / 1e9
            potential_ang_velocities = convert_serial_data_to_angular_velocities(serial_read_back, self.get_logger())
            if potential_ang_velocities is not None:
                ang_velocities = potential_ang_velocities
                robot_velocities = self.wheel.calculate_robot_velocities(ang_velocities)
                self.update_position_with_odometry(delta_time, robot_velocities)
                odom = fill_odometry_message(self.x, self.y, self.theta, current_time, robot_velocities)
                var_gearbox_backlash = var_gearbox_backlash(ang_velocities, np.radians(0.5), self.wheel)

                variances = self.var_encoding + var_gearbox_backlash
                # odom.pose.covariance = [variances[0], 0, 0, 0, 0, 0,
                #                         0, variances[1], 0, 0, 0, 0,
                #                     0, 0, 99999, 0, 0, 0,
                #                     0, 0, 0, 99999, 0, 0,
                #                     0, 0, 0, 0, 99999, 0,
                #                     0, 0, 0, 0, 0, variances[2]]  

                # odom.twist.covariance = [variances[0], 0, 0, 0, 0, 0,
                #                      0, variances[1], 0, 0, 0, 0,
                #                      0, 0, 99999, 0, 0, 0,
                #                      0, 0, 0, 99999, 0, 0,
                #                      0, 0, 0, 0, 99999, 0,
                #                      0, 0, 0, 0, 0, variances[2]]  

                self.odom_publisher.publish(odom)
                self.tf_publish(current_time)
                self.last_time - current_time

    def update_position_with_odometry(self, delta_time, robot_velocities):
        delta_x = robot_velocities[0] * delta_time
        delta_y = robot_velocities[1] * delta_time
        delta_theta = robot_velocities[2] * delta_time

        self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
        self.y += delta_x * math.cos(self.theta) + delta_y * math.sin(self.theta)
        self.theta += delta_theta

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublishing()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
