import rclpy
from geometry_msgs.msg import PoseStamped
import time

class GoalPublisher:
    def __init__(self):
        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('goal_publisher')

        # Create a publisher for PoseStamped messages
        self.goal_publisher = self.node.create_publisher(PoseStamped, 'goal_pose', 10)

        # # Wait for the publisher to be ready
        # while not self.goal_publisher.publisher_is_ready():
        #     time.sleep(0.1)

    def publish_goal(self, x, y, z, qx, qy, qz, qw):
        # Create a PoseStamped message
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.x = qx
        goal_msg.pose.orientation.y = qy
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        # Publish the goal
        self.goal_publisher.publish(goal_msg)

    def run(self):
        # Example: Publish a goal
        self.publish_goal(10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        # Run the ROS2 node
        rclpy.spin(self.node)

def main(args=None):
    goal_publisher = GoalPublisher()
    goal_publisher.run()

if __name__ == '__main__':
    main()
