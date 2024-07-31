import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot3_interfaces.srv import FindClosestWall  # Change to your actual service import

class WallFinderService(Node):
    def __init__(self):
        super().__init__('wall_finder_service')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.srv = self.create_service(FindClosestWall, 'find_closest_wall', self.find_closest_wall_callback)
        self.closest_wall_distance = 0.3  # Distance to consider as reached the wall
        self.reached_wall = False

    def find_closest_wall_callback(self, request, response):
        self.reached_wall = False
        while not self.reached_wall:
            rclpy.spin_once(self)
        response.success = True
        return response

    def scan_callback(self, msg):
        front_distance = min(msg.ranges[0:10] + msg.ranges[-10:])

        twist = Twist()
        if front_distance > self.closest_wall_distance:
            twist.linear.x = 0.1
        else:
            twist.linear.x = 0.0
            self.reached_wall = True

        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WallFinderService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
