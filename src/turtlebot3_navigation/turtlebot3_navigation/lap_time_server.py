import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from turtlebot3_interfaces.action import MeasureLapTime
from nav_msgs.msg import Odometry
import time

class LapTimeActionServer(Node):
    def __init__(self):
        super().__init__('lap_time_action_server')
        self._action_server = ActionServer(
            self,
            MeasureLapTime,
            'lap_time',
            self.execute_callback
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.start_position = None
        self.current_position = None
        self.start_time = None

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose
        # self.get_logger().info(f'current position: {self.current_position}')
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Starting lap timer...')

        self.start_time = self.get_clock().now()
        self.start_position = self.current_position
        # self.get_logger().info(f'start position: {self.start_position}')

        
        while not self.is_lap_completed():
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            feedback_msg = MeasureLapTime.Feedback()
            feedback_msg.elapsed_time = elapsed_time
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  
        total_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        result = MeasureLapTime.Result()
        result.total_time = total_time
        goal_handle.succeed()

        self.get_logger().info(f'Lap completed in {total_time} seconds')
        return result

    def is_lap_completed(self):
        if self.start_position and self.current_position and (self.get_clock().now() - self.start_time).nanoseconds / 1e9 > 20:
            distance = self.calculate_distance(self.start_position, self.current_position)
            #self.get_logger().info(f'distance: {distance}')
            if distance < 0.3:  
                return True
        return False

    def calculate_distance(self, start, current):
        dx = current.position.x - start.position.x
        dy = current.position.y - start.position.y
        dz = current.position.z - start.position.z
        return (dx**2 + dy**2 + dz**2) ** 0.5

def main(args=None):
    rclpy.init(args=args)
    action_server = LapTimeActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
