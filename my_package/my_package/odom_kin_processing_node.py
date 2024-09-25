import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import serial 
import struct
from kinematics_mechanum_wheel import KinematicMechanumWheel

class KinOdomProcessing(Node):

    def __init__(self) -> None:
        super().__init__('kinematics_processing_subscriber_publisher')
        self.subscription = self.create_subscription(Twist, 'turtle1/cmd_vel', self.kinematics_callback, 10)
        y_to_wheel = (15/100)
        x_to_wheel =  (15/100)
        radius = ((8/2)/100) 
        self.max_angular_velocities = 60
        self.max_output_angular_velocities = 11
        angle_from_wheels = np.pi/2
        self.wheel = KinematicMechanumWheel(y_to_wheel, x_to_wheel, radius, angle_from_wheels)
        self.serial = serial.Serial("/dev/Serial0", 9600)