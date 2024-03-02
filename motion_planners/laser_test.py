import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MoveAndStopNode(Node):
    def __init__(self):
        super().__init__('move_and_stop_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Replace with your laser scan topic
            self.laser_scan_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',  # Replace with your robot's velocity command topic
            10)

        self.twist_msg = Twist()

    def laser_scan_callback(self, msg):
        # Assuming laser scan is in a horizontal orientation and obstacle detection is at a specific range
        if min(msg.ranges[150:250]) <= 1.0:  # Replace 1.0 with your desired stop distance
            self.twist_msg.linear.x = 0.0
        else:
            self.twist_msg.linear.x = 1.0  # Replace 0.2 with your desired forward speed

        self.publisher_.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)

    move_and_stop_node = MoveAndStopNode()

    rclpy.spin(move_and_stop_node)

    move_and_stop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
