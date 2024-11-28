import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from motor_control import MotorControl  # Assume MotorControl is saved in motor_control.py

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        self.odom_pub = self.create_publisher(Odometry,'odom',10)
        self.timer = self.create_timer(0.1,self.odomdata)

    def odomdata(self,msg):
        msg = Odometry()
        msg.motor1_fb =  MotorControl.get_motor_feedback(2)
        msg.motor2_fb =  MotorControl.get_motor_feedback(3)

        self.get_logger().info("")

        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WheelOdom())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
