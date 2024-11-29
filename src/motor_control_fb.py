import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from motor_control import MotorControl  # Assume MotorControl is saved in motor_control.py
import math
import time


class WheelControl(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        # Parameters
        self.declare_parameter('wheel_radius', 0.04)  # Wheel radius in meters
        self.declare_parameter('wheel_base', 0.482)  # Distance between wheels in meters

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        # Initialize odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Serial settings
        self.motor_control = MotorControl(device="/dev/ttyUSB0")

        # Publishers and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        ## cmd_Vel sub
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_Vel_Callback,
            10
        )
        self.last_time = time.time()

        # Timer to periodically publish odometry
        self.timer = self.create_timer(0.1, self.odomdata)

    def odomdata(self):
        # Get current time and calculate time difference
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        try:
            # Get feedback from motors (remotor_fbplace with actual implementation)
            left_rpm, left_current = self.motor_control.get_motor_feedback(1)
            right_rpm, right_current = self.motor_control.get_motor_feedback(2)

            # Convert RPM to linear velocity (m/s)
            v_left = (left_rpm * 2 * math.pi * self.wheel_radius) / 60.0
            v_right = ((-right_rpm) * 2 * math.pi * self.wheel_radius) / 60.0

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
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = quaternion
            self.tf_broadcaster.sendTransform(transform)

        except Exception as e:
            self.get_logger().error(f"Error in odometry calculation: {e}")

    def cmd_Vel_Callback(self,msg:Twist):
        linear_x = msg.linear.x  # Linear velocity (m/s)
        angular_z = msg.angular.z  # Angular velocity (rad/s)
        

        # Convert to wheel velocities (m/s)
        left_wheel_velocity = linear_x - (angular_z * self.wheel_base / 2.0)
        right_wheel_velocity = linear_x + (angular_z * self.wheel_base / 2.0)

        # Convert to RPM (assume wheel circumference and unit conversions)
        left_rpm = (left_wheel_velocity / (2 * 3.1416 * self.wheel_radius)) * 60
        right_rpm = (right_wheel_velocity / (2 * 3.1416 * self.wheel_radius)) * 60

        # Send RPM commands to motors
        try:
            self.motor_control.send_rpm(1, left_rpm)  # Motor ID 1 for left wheel
            self.motor_control.send_rpm(2, -right_rpm)  # Motor ID  for right wheel
            self.get_logger().info(f"Set left RPM: {left_rpm}, right RPM: {right_rpm}")

            leftwheel_rpm,current_fb =self.motor_control.get_motor_feedback(1)    # Motor ID 1 feedback 
            rightwheel_rpm,current_fb =self.motor_control.get_motor_feedback(2)    # Motor ID 2 feedback
            self.get_logger().info(f"leftwheelrpm:{leftwheel_rpm},rithwheel:{rightwheel_rpm}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to send RPM: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WheelControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
