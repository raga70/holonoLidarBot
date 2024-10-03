import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from inputs import get_gamepad
import math
import threading

class XboxController(object):
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickX = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        x = self.LeftJoystickX
        y = self.LeftJoystickY
        right_x = self.RightJoystickX
        return [x, y, right_x]

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1

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
        self.X = joystick_values[0]  # Left joystick X controls linear.y
        self.Y = joystick_values[1]  # Left joystick Y controls linear.x
        self.A = joystick_values[2]  # Right joystick X controls angular.z

        # Publish Twist message
        msg = Twist()
        msg.linear.x = -float(self.Y)
        msg.linear.y = float(self.X)
        msg.angular.z = float(self.A)
        self.publisher_.publish(msg)

def main(args=None):
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
