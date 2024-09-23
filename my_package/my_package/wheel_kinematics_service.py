import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import serial 
import struct
from kinematics_mechanum_wheel import KinematicMechanumWheel

class KinematicsProcessing(Node):

    def __init__(self) -> None:
        super().__init__('kinematics_processing_subscriber_publisher')
        self.subscription = self.create_subscription(Twist, 'turtle1/cmd_vel', self.kinematics_callback, 10)
        y_to_wheel = (15/100)
        x_to_wheel =  (15/100)
        radius = ((8/2)/100) 
        angle_from_wheels = np.pi/2
        self.wheel = KinematicMechanumWheel(y_to_wheel, x_to_wheel, radius, angle_from_wheels)
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

    
        


