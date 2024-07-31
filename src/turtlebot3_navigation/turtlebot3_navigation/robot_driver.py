import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from turtlebot3_interfaces.srv import FindClosestWall
from rclpy.executors import MultiThreadedExecutor
import time
from turtlebot3_interfaces.srv import LapStartingTime



class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.callback_group = ReentrantCallbackGroup()

        self.client = self.create_client(
            FindClosestWall, 'find_closest_wall', callback_group=self.callback_group
        )
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10, callback_group=self.callback_group
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.srv = self.create_service(LapStartingTime, 'check_response', self.handle_check_response, callback_group=self.callback_group)
        self.node_lap_client_ready = False 

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Service not available, waiting...')

        self.closest_distance = float('inf')
        self.closest_angle = None
        self.first_index = None
        self.front_distance = None
        self.left_index = None
        self.left_angle = None
        self.left_distance = None
        
        
        self.call_wall_finder_service()
        
        
    def handle_check_response(self, request, response):
        
        response.success = False
        if self.node_lap_client_ready == True:
            response.success = True
            
        return response
        
    def call_wall_finder_service(self):
        request = FindClosestWall.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully found the closest wall.')
                self.follow_wall()
            else:
                self.get_logger().info('Failed to find the closest wall.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def scan_callback(self, msg):
        """
        Callback function to process LaserScan messages and follow the wall.
        """
        # Get the distance to the wall from the LaserScan data
        ranges = msg.ranges
        self.closest_distance = min(ranges)
        min_index = ranges.index(self.closest_distance)
        angle_increment = msg.angle_increment
        self.closest_angle = msg.angle_min + min_index * angle_increment
        self.first_index = msg.angle_min

        self.front_distance = msg.ranges[0]
        self.left_index = msg.angle_min + 1.5708
        
        self.left_distance = ranges[int(len(ranges) / 4)]  # 90 degrees to the left

        # self.get_logger().info(f'Closest wall at distance: {self.closest_distance}, angle: {self.closest_angle}, front distance: {self.front_distance}')

    def follow_wall(self):
        """
        Method to follow the wall.
        """
       
        while self.left_distance is None and self.first_index is None and self.front_distance is None and self.closest_angle is None:
            self.get_logger().info('waiting for closest index, first index and front distance')
        
            
        self.get_logger().info('Following Wall')
        twist = Twist()

        while (True):
            
            if self.front_distance < self.left_distance :
                while abs(self.first_index - self.closest_angle) < 1.5708:
                    #self.get_logger().info(f'first index - closest angle = {self.first_index - self.closest_angle}')
                    twist.angular.z = -0.2
                    self.publisher_.publish(twist)

                
            elif self.front_distance > self.left_distance:
                while abs(self.left_index - self.closest_angle) < 1.5708:
                    #self.get_logger().info(f'first index - closest angle = {self.left_index - self.closest_angle}')
                    twist.angular.z = -0.2
                    self.publisher_.publish(twist)
            
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
            self.node_lap_client_ready = True
            
                
            while self.front_distance > 0.5:
                twist.linear.x = 0.2
                self.publisher_.publish(twist) 
            
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
          
        

        

        
        
        
def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    
    # Use MultiThreadedExecutor to handle callbacks in separate threads
    executor = MultiThreadedExecutor()
    executor.add_node(robot_driver)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        robot_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




