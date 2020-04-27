from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import time
import random


class MinimalActionClient():

    def __init__(self, action_type, node, server_name):
        self._server_name = server_name
        if action_type=="Fibonacci":
            self._action_client = ActionClient(node, Fibonacci, server_name)

        self.__result = None
        self.__state = "Free"

    def get_result(self):
        return self.__result

    def set_result(self, val):
        self.__result = val

    def get_state(self):
        return self.__state

    def set_state(self, new_state):
        self.__state = new_state

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
        if self._server_name == "fibonacci_1":
            self.__result = 't1'
        elif self._server_name == "fibonacci_2":
            self.__result = None
        elif self._server_name == "fibonacci_3":
            self.__result = None
        elif self._server_name == "fibonacci_4":
            self.__result = 't3'
        elif self._server_name == "fibonacci_5":
            self.__result = 't3'
        elif self._server_name == "fibonacci_6":
            self.__result = 't3'
        elif self._server_name == "fibonacci_7":
            self.__result = 't4'
        elif self._server_name == "fibonacci_8":
            self.__result = 't4'
        elif self._server_name == "fibonacci_9":
            self.__result = None
        elif self._server_name == "fibonacci_10":
            self.__result = 't6'


        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print(self._server_name+': Goal succeeded! Result: {0}'.format(result.sequence))
            self.__state = "Done"
        else:
            print(self._server_name+': Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        # rclpy.shutdown()

    def send_goal(self, order):
        # self.get_logger().info('Waiting for action server '+self._server_name)
        print('Waiting for action server '+self._server_name)
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # self.get_logger().info('Sending goal request to '+self._server_name)
        print('Sending goal request to '+self._server_name)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    client_node = rclpy.create_node('minimal_action_client')
    ac_list = []

    action_client_1 = MinimalActionClient(node=client_node, server_name='fibonacci_1')
    action_client_2 = MinimalActionClient(node=client_node, server_name='fibonacci_2')

    ac_list.append(action_client_1)
    ac_list.append(action_client_2)
    # action_client_1.send_goal()
    # action_client_2.send_goal()

    while True:
        for i in range(len(ac_list)):

            if ac_list[i].goal_done():
                print("I'm done")
                print("result", ac_list[i].get_result())
                ac_list[i].set_done(False)
                ac_list[i].set_result(None)
                ac_list[i].set_free(True)
                print("apply policy")

            else:
                if ac_list[i].client_free():
                    ac_list[i].set_free(False)
                    if i == 0:
                        ac_list[i].send_goal(5)
                    else:
                        ac_list[i].send_goal(6)
                    print("sent goal")
                else:
                    rclpy.spin_once(client_node)
                    print("spinning")

    # rclpy.spin(client_node)
    print("print executed")


    # client_node.destroy()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
