import time

from action_package.action import Simple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server_6')

        self._action_server = ActionServer(
            self,
            Simple,
            'simple_6',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('SERVER 6 : ONLINE')

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('SERVER 6 : Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info('SERVER 6 : Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Executes a goal."""
        self.get_logger().info('SERVER 6 : Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = Simple.Feedback()
        feedback_msg.time_passed = [0]

        # Start executing the action
        for i in range(1, 5):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('SERVER 6 : Goal canceled')
                return Simple.Result()

            # Update Fibonacci sequence
            feedback_msg.time_passed.append(i)

            self.get_logger().info('SERVER 6 : Publishing feedback: {0}'.format(feedback_msg.time_passed))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = Simple.Result()
        result.transition = 't8'

        self.get_logger().info('SERVER 6 : Returning result: {0}'.format(result.transition))

        return result


def main(args=None):
    rclpy.init(args=args)
    minimal_action_server = MinimalActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
