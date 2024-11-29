import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from motor_control import MotorControl  # Assume MotorControl is saved in motor_control.py
import math
import time


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        # Parameters
        self.declare_parameter('wheel_radius', 0.04)  # Wheel radius in meters
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels in meters

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        # Initialize odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Serial settings
        self.motor_control = MotorControl(device="/dev/ttyACM0")

        # Publishers and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_time = time.time()

        # Timer to periodically publish odometry
        self.timer = self.create_timer(0.1, self.odomdata)

    def odomdata(self):
        # Get current time and calculate time difference
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        try:
            # Get feedback from motors (replace with actual implementation)
            left_rpm, left_current = self.motor_control.get_motor_feedback('left')
            right_rpm, right_current = self.motor_control.get_motor_feedback('right')

            # Convert RPM to linear velocity (m/s)
            v_left = (left_rpm * 2 * math.pi * self.wheel_radius) / 60.0
            v_right = (right_rpm * 2 * math.pi * self.wheel_radius) / 60.0

            # Calculate robot velocities
            v = (v_left + v_right) / 2.0
            omega = (v_right - v_left) / self.wheel_base

            # Update position
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += omega * dt

            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

            # Create odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'

            # Pose
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            quaternion = Quaternion()
            quaternion.z = math.sin(self.theta / 2.0)
            quaternion.w = math.cos(self.theta / 2.0)
            odom.pose.pose.orientation = quaternion

            # Twist
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = omega

            # Publish odometry
            self.odom_pub.publish(odom)

            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = quaternion
            self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.get_logger().error(f"Error in odometry calculation: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
