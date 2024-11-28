import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_control import MotorControl  # Assume MotorControl is saved in motor_control.py

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.declare_parameter('wheel_radius', 0.04)  # Wheel radius in meters
        self.declare_parameter('wheel_base', 0.2)  # Distance between wheels in meters

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Initialize motor control
        self.motor_control = MotorControl(device="/dev/ttyACM0")

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('MotorControllerNode initialized.')

    def cmd_vel_callback(self, msg: Twist):
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
            self.motor_control.send_rpm(2, right_rpm)  # Motor ID  for right wheel
            self.get_logger().info(f"Set left RPM: {left_rpm}, right RPM: {right_rpm}")

            leftwheel_rpm,current_fb =self.motor_control.get_motor_feedback(1)    # Motor ID 1 feedback 
            rightwheel_rpm,current_fb =self.motor_control.get_motor_feedback(2)    # Motor ID 2 feedback
            self.get_logger().info(f"leftwheelrpm:{leftwheel_rpm},rithwheel:{rightwheel_rpm}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to send RPM: {e}")

    def destroy_node(self):
        self.motor_control.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        motor_controller_node.get_logger().info('Shutting down.')
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
