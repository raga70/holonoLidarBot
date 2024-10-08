import pygame
import time
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from inputs import get_gamepad
import math
import threading
import pygame



class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    DEADZONE = 0.15  # Define the deadzone threshold (5% of max value)

    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickX = 0
        self.joystick = pygame.joystick.Joystick(0)

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
    
    def read(self):
        print(self.LeftJoystickX,self.LeftJoystickY,self.RightJoystickX)
        x = self.apply_deadzone(self.LeftJoystickX)
        y = self.apply_deadzone(self.LeftJoystickY)
        right_x = self.apply_deadzone(self.RightJoystickX)
        return [x, y, right_x]

    def apply_deadzone(self, value):
        if abs(value) < XboxController.DEADZONE:
            return 0.0
        return value

    def _monitor_controller(self):
        while True:
            pygame.event.pump()
            self.LeftJoystickY = self.joystick.get_axis(1)   # normalize between -1 and 1
            self.LeftJoystickX = self.joystick.get_axis(0)   # normalize between -1 and
            self.RightJoystickX = self.joystick.get_axis(3)   # normalize between -1 and 1
            time.sleep(0.1)
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.X = 0.0
        self.Y = 0.0
        self.A = 0.0

        # Initialize Xbox controller
        self.joystick = XboxController()
 

    def timer_callback(self):
        # Get joystick inputs
        joystick_values = self.joystick.read()
        
        self.X = joystick_values[1]  # Left joystick X controls linear.y
        self.Y = joystick_values[0]  # Left joystick Y controls linear.x
        self.A = joystick_values[2]  # Right joystick X controls angular.z

        # Publish Twist message
        msg = Twist()
        msg.linear.x = -float(self.X)
        msg.linear.y = -float(self.Y)
        msg.angular.z = -float(self.A)
        self.publisher_.publish(msg)
        

def main(args=None):
    print("Xbox contol ACTIVE!")
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
