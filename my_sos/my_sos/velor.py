import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.sign_subscription = self.create_subscription(
            Int32,
            '/ride_or_not',
            self.sign_callback,
            10
        )

        # self.angle_subscription = self.create_subscription(
        #     Float32,
        #     '/angle',
        #     self.angle_callback,
        #     10
        # )

        self.angle = 0

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def angle_callback(self, msg):
        self.angle = msg.data

    def sign_callback(self, msg):
        if msg.data == 1:   # we need to go strict forward
            cmd_vel_msg = Twist()
        
            linear_speed = 0.2
            angular_speed = 10 * self.angle
            
            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = float(angular_speed)

            self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()