from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MinimalActionClient():

    def __init__(self, node, server_name):
        self._server_name = server_name
        self._action_client = ActionClient(node, Fibonacci, server_name)

        self.__result = None
        self.__done = False

    def goal_done(self):
        return self.__done

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :( '+self._server_name)
            return

        print('Goal accepted :) '+self._server_name)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        print('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        print("RESULT", result)
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print(self._server_name+': Goal succeeded! Result: {0}'.format(result.sequence))
            self.__done = True
        else:
            print(self._server_name+': Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        # rclpy.shutdown()

    def send_goal(self):
        # self.get_logger().info('Waiting for action server '+self._server_name)
        print('Waiting for action server '+self._server_name)
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        # self.get_logger().info('Sending goal request to '+self._server_name)
        print('Sending goal request to '+self._server_name)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    client_node = rclpy.create_node('minimal_action_client')

    action_client_1 = MinimalActionClient(node=client_node, server_name='fibonacci_1')
    action_client_2 = MinimalActionClient(node=client_node, server_name='fibonacci_2')

    action_client_1.send_goal()
    action_client_2.send_goal()

    rclpy.spin(client_node)

    # client_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
