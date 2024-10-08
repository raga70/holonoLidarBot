import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_from_euler

class KinematicsProcessing(Node):
    def __init__(self):
        super().__init__('kinematics_processing_subscriber_publisher')
        
        # Subscriber for the /cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.kinematics_callback,
            10
        )

        # Publisher for wheel velocities (optional)
        self.wheel_vel_publisher = self.create_publisher(Twist, 'wheel_velocities', 10)

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster to send odom -> base_link transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize robot position and orientation
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_orientation = 0.0  # Yaw angle in radians

        # Initialize velocity storage (x, y, angular_z)
        self.current_velocity = np.array([0.0, 0.0, 0.0])

        # Timer for periodically publishing odometry
        self.timer = self.create_timer(0.1, self.publish_odometry)

    def kinematics_callback(self, msg):
        """Callback for /cmd_vel messages."""
        # Extract linear and angular velocities from the message
        velocities = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

        # Here you could process these velocities and calculate the wheel velocities
        # This is an example where you directly publish velocities as received (for now)
        self.current_velocity = velocities

        # You could publish these wheel velocities if needed
        self.publish_wheel_velocities(velocities)

        # Update robot pose using simple odometry (this is a very basic model)
        dt = 0.1  # Assuming this callback is called at 10 Hz

        # Update the robot's position and orientation (you can use better kinematic models)
        self.robot_pose_x += velocities[0] * dt
        self.robot_pose_y += velocities[1] * dt
        self.robot_orientation += velocities[2] * dt

        # Broadcast the TF transform (odom -> base_link)
        self.broadcast_transform()

    def publish_wheel_velocities(self, velocities):
        """Publish the wheel velocities (optional)."""
        wheel_vel_msg = Twist()
        wheel_vel_msg.linear.x = velocities[0]
        wheel_vel_msg.linear.y = velocities[1]
        wheel_vel_msg.angular.z = velocities[2]
        
        self.wheel_vel_publisher.publish(wheel_vel_msg)

    def broadcast_transform(self):
        """Broadcast the odom -> base_link transform."""
        t = TransformStamped()

        # Set the frame names and timestamps
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Set the translation (robot's position)
        t.transform.translation.x = self.robot_pose_x
        t.transform.translation.y = self.robot_pose_y
        t.transform.translation.z = 0.0

        # Convert yaw to quaternion and set the rotation
        q = quaternion_from_euler(0, 0, self.robot_orientation)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def publish_odometry(self):
        """Publish the odometry data."""
        odom_msg = Odometry()

        # Set the frame and timestamp
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set the position (robot's pose)
        odom_msg.pose.pose.position.x = self.robot_pose_x
        odom_msg.pose.pose.position.y = self.robot_pose_y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation (from yaw to quaternion)
        q = quaternion_from_euler(0, 0, self.robot_orientation)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set the velocities (linear and angular)
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.angular.z = self.current_velocity[2]

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    kinematics_processing = KinematicsProcessing()

    # Spin the node so its callbacks are active
    rclpy.spin(kinematics_processing)

    # Shutdown the node
    kinematics_processing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
