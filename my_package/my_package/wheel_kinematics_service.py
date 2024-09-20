import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import serial 
import struct

class BckKinematicMechanumWheel:

    def __init__(self, y_to_wheel, x_to_wheel, wheel_radius, angle_from_wheels):
        self.y_to_wheel = y_to_wheel
        self.x_to_wheel = x_to_wheel
        self.wheel_radius = wheel_radius
        self.beta = angle_from_wheels
        self.alpha = np.arctan(self.y_to_wheel / self.x_to_wheel)
        self.yetas = np.array([np.radians(-45), np.radians(45), np.radians(45), np.radians(-45)])
        self.length = np.sqrt(self.y_to_wheel**2 + self.x_to_wheel**2)

        self.T_wo_r = np.array([
            [np.cos(self.beta - self.yetas[0])/np.sin(self.yetas[0]), np.sin(self.beta - self.yetas[0])/np.sin(self.yetas[0]), self.length * np.sin(self.beta - self.yetas[0] - self.alpha)/np.sin(self.yetas[0])],
            [np.cos(self.beta - self.yetas[1])/np.sin(self.yetas[1]), np.sin(self.beta - self.yetas[1])/np.sin(self.yetas[1]), self.length * np.sin(self.beta - self.yetas[1] - self.alpha)/np.sin(self.yetas[1])],
            [np.cos(self.beta - self.yetas[2])/np.sin(self.yetas[2]), np.sin(self.beta - self.yetas[2])/np.sin(self.yetas[2]), self.length * np.sin(self.beta - self.yetas[2] - self.alpha)/np.sin(self.yetas[2])],
            [np.cos(self.beta - self.yetas[3])/np.sin(self.yetas[3]), np.sin(self.beta - self.yetas[3])/np.sin(self.yetas[3]), self.length * np.sin(self.beta - self.yetas[3] - self.alpha)/np.sin(self.yetas[3])],
        ])

        self.T = 1/self.wheel_radius * self.T_wo_r

    def calculate_wheel_velocities(self, velocities):
        return self.T @ velocities

class KinematicsProcessing(Node):

    def __init__(self) -> None:
        super().__init__('kinematics_processing_subscriber_publisher')
        self.subscription = self.create_subscription(Twist, 'turtle1/cmd_vel', self.kinematics_callback, 10)
        y_to_wheel = (15/100)
        x_to_wheel =  (15/100)
        radius = ((8/2)/100) 
        angle_from_wheels = np.pi/2
        self.wheel = BckKinematicMechanumWheel(y_to_wheel, x_to_wheel, radius, angle_from_wheels)
        self.serial = serial.Serial("/dev/ttyAMA0", 9600)
        #dev/Serial0

    def kinematics_callback(self, msg):
        velocities = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z]
            )
        wheel_ang_velocities = self.wheel.calculate_wheel_velocities(velocities)
        self.get_logger().info(f"{wheel_ang_velocities}")
        # 1st motor front left first value 
        # 2nd motor front right second
        # 3rd motor back left third
        # 4th motor back right fourth

        for i in range(4):
            serial_message = bytearray(["m ", f"{i} ", f"{wheel_ang_velocities[i]}", "\n"])
            self.serial.write(serial_message)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsProcessing()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    
        


