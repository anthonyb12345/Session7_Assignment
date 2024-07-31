import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot3_interfaces.srv import FindClosestWall  # Change to your actual service import

class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.client = self.create_client(FindClosestWall, 'find_closest_wall')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.call_find_closest_wall_service()
        self.wall_distance = 0.5  # Desired distance from the wall

    def call_find_closest_wall_service(self):
        request = FindClosestWall.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.find_closest_wall_response_callback)

    def find_closest_wall_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Reached closest wall successfully.')
            else:
                self.get_logger().warn('Failed to reach the closest wall.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def scan_callback(self, msg):
        # Basic wall following algorithm
        front_distance = min(msg.ranges[0:10] + msg.ranges[-10:])
        left_distance = min(msg.ranges[60:120])
        right_distance = min(msg.ranges[-120:-60])

        twist = Twist()

        if left_distance < self.wall_distance:
            twist.angular.z = -0.2
        elif left_distance > self.wall_distance:
            twist.angular.z = 0.2

        twist.linear.x = 0.1
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
