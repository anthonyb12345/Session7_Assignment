import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot3_interfaces.srv import FindClosestWall
from rclpy.executors import MultiThreadedExecutor
import time

class WallFinderServer(Node):
    def __init__(self):
        super().__init__('wall_finder_server')

        # Create a ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            FindClosestWall, 'find_closest_wall', self.find_closest_wall_callback, callback_group=self.callback_group
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10, callback_group=self.callback_group
        )

        self.closest_distance = float('inf')
        self.closest_angle = None
        self.first_index = None
        

    def scan_callback(self, msg):
        """
        Callback function to process LaserScan messages.
        """
        ranges = msg.ranges
        self.closest_distance = min(ranges)
        min_index = ranges.index(self.closest_distance)
        angle_increment = msg.angle_increment
        self.closest_angle = msg.angle_min + min_index * angle_increment
        self.first_index = msg.angle_min
        #self.get_logger().info(f'Closest wall at distance: {self.closest_distance}, angle: {self.closest_angle}')

    def find_closest_wall_callback(self, request, response):
        """
        Service callback to handle wall finding logic.
        """
        while self.closest_angle is None and self.first_index is None:
            pass
            # self.get_logger().info('waiting for closest index and first index')
        
            
        self.get_logger().info('Finding the closest wall...')
        twist = Twist()
        time.sleep(5)
        
        twist.linear.x = 0.2
        self.publisher_.publish(twist)

        time.sleep(1)

        twist.linear.x = 0.0
        self.publisher_.publish(twist)

        while abs(self.first_index - self.closest_angle) > 0.1 :
            #self.get_logger().info(f'first index - closest angle = {self.first_index - self.closest_angle}')
            twist.angular.z = 0.2
            self.publisher_.publish(twist)

        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        
        while self.closest_distance > 0.5:
            twist.linear.x = 0.2
            self.publisher_.publish(twist)
        
        twist.linear.x = 0.0
        self.publisher_.publish(twist)

        self.get_logger().info('Wall found and robot stopped.')
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WallFinderServer()
    
    # Use MultiThreadedExecutor to handle callbacks in separate threads
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()










