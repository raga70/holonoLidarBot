import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjusted timer for smoother control
        self.X = 0.0
        self.Y = 0.0
        self.A = 0.0
        self.speed_step = 1.0  # Change this to the desired increment/decrement value
        self.angular_step = 0.314  # In radians

        # Flag to control key inputs
        self.allow_key_input = True

        # Start the keyboard listener
        self.listener = keyboard.Listener(suppress=True,on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.Y
        msg.linear.y = self.X
        msg.angular.z = self.A
        self.publisher_.publish(msg)

    def on_press(self, key):
        if not self.allow_key_input:
            return  # Ignore key presses if not allowed

        try:
            if key.char == 'w':
                self.Y = self.speed_step
            elif key.char == 's':
                self.Y = -self.speed_step
            elif key.char == 'a':
                self.X = self.speed_step
            elif key.char == 'd':
                self.X = -self.speed_step
            elif key.char == 'q':
                self.A = self.angular_step
            elif key.char == 'e':
                self.A = -self.angular_step
        except AttributeError:
            pass
            
    def on_release(self, key):
        if not self.allow_key_input:
            return  # Ignore key releases if not allowed

        try:
            if key.char == 'w' or key.char == 's':
                self.Y = 0.0
            elif key.char == 'a' or key.char == 'd':
                self.X = 0.0
            elif key.char == 'q' or key.char == 'e':
                self.A = 0.0
        except AttributeError:
            pass

    def disable_keys(self):
        self.allow_key_input = False

    def enable_keys(self):
        self.allow_key_input = True

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    # Here you can call disable_keys or enable_keys as needed
    # For example, after a certain event, you might call:
    # minimal_publisher.disable_keys()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.listener.stop()
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
