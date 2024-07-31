import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlebot3_interfaces.action import MeasureLapTime
from turtlebot3_interfaces.srv import LapStartingTime
import time

class LapTimeActionClient(Node):
    def __init__(self):
        super().__init__('lap_time_action_client')
        self._action_client = ActionClient(self, MeasureLapTime, 'lap_time')
        self.client = self.create_client(
            LapStartingTime, 'check_response'
        )
        # Wait for the service to become available
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
            #self.get_logger().info('Service not available, waiting...')
        
        self.call_lap_starting_time()
    
    def call_lap_starting_time(self):
        request = LapStartingTime.Request()
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Sending Goal')
                self.send_goal()
            else:
                #self.get_logger().info('Failed to send goal.')
                time.sleep(1)
                self.call_lap_starting_time()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_goal(self):
        goal_msg = MeasureLapTime.Goal()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Elapsed time: {feedback.elapsed_time} seconds')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Lap completed in {result.total_time} seconds')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = LapTimeActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
