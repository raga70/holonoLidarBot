from typing import Optional
from geometry_msgs.msg import Quaternion
import numpy as np
from nav_msgs.msg import Odometry
from math_utils import euler_to_quaternion
def convert_serial_data_to_angular_velocities(serial_read_back: bytes, logger):
    split_data = str(serial_read_back)[2:-1].split(',')
    try:
        omega_1, omega_2, omega_3, omega_4 = map(float, split_data)
        if logger is not None:
            logger.info(f"split data: {split_data}")
        ang_velocities = np.array([omega_1, omega_2, omega_3, omega_4])
        return ang_velocities
    except Exception as e:
        print("skipped updating odometry data")
        print(e)
        return None

def fill_odometry_message(x_pos, y_pos, theta, current_time, robot_velocities) -> Odometry:
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x_pos
        odom.pose.pose.position.y = y_pos
        odom.pose.pose.position.z = 0.0

        orientation = euler_to_quaternion(0, 0, theta)
        odom.pose.pose.orientation = Quaternion()
        odom.pose.pose.orientation.x = orientation[1]
        odom.pose.pose.orientation.y = orientation[2]
        odom.pose.pose.orientation.z = orientation[3]
        odom.pose.pose.orientation.w = orientation[0]

        odom.twist.twist.linear.x = robot_velocities[0]
        odom.twist.twist.linear.y = robot_velocities[1]
        odom.twist.twist.angular.z = robot_velocities[2]
        return odom
