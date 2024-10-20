import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MonitoringMode(Node):

    def __init__(self):
        super().__init__('monitoring_mode')
        self.control_publisher = self.create_publisher(Twist, '/control', 10)
        self.yaw_speed = 50.0
        self.timer = self.create_timer(1.0, self.publish_yaw_command) # Set the publishing frequency (1Hz)

    def publish_yaw_command(self):
        """Callback function to continuously publish the yaw command at 1Hz."""
        twist = Twist()
        twist.angular.z = self.yaw_speed
        self.control_publisher.publish(twist)
        self.get_logger().info(f'Publishing continuous yaw command: {self.yaw_speed}')


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringMode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
