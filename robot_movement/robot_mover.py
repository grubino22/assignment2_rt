import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class Robot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.move_robot()

    def move_robot(self):
        velocity = Twist()
        if 2.0 <= self.position_x <= 9.0:
            velocity.linear.x = 1.0
            velocity.angular.z = 0.0
        elif self.position_x > 9.0:
            velocity.linear.x = 1.0
            velocity.angular.z = 1.57
        elif self.position_x < 2.0:
            velocity.linear.x = 1.0
            velocity.angular.z = -1.57
        
        self.publisher_.publish(velocity)
        self.get_logger().info(f'Moving robot: {velocity.linear.x} m/s, {velocity.angular.z} rad/s')

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

