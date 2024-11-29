import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")
        
        # Parameters
        self.declare_parameter("wheel_separation", 0.48)  # Distance between wheels (m)
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        # Initialize odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity feedback (example values; replace with real motor feedback)
        self.v_left = 0.0  # m/s
        self.v_right = 0.0  # m/s

        self.last_time = time.time()

        # Publisher and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish odometry at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_odometry)

    def publish_odometry(self):
        # Calculate dt
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Compute linear and angular velocities
        v = (self.v_left + self.v_right) / 2.0
        omega = (self.v_right - self.v_left) / self.wheel_separation

        # Update position and orientation
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create quaternion from theta
        quaternion = Quaternion()
        quaternion.z = math.sin(self.theta / 2.0)
        quaternion.w = math.cos(self.theta / 2.0)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion

        # Twist
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # Publish transform (for tf2)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
