import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ast import literal_eval

class gspn_executor(Node):

    def __init__(self, robot_id):
        node_name="gspn_executor_" + str(robot_id)
        super().__init__(node_name, namespace="robot_" + str(robot_id))
        self.publisher = self.create_publisher(String, '/TRANSITIONS_FIRED', 10)
        self.subscription = self.create_subscription(String, '/TRANSITIONS_FIRED', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.__transitions_fired = []
        self.i = 0


    def talker_callback(self, fired_transition, robot_id):
        msg = String()
        msg.data = str([fired_transition, robot_id])
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def get_transitions_fired(self):
        return self.__transitions_fired

    def reset_transitions_fired(self):
        self.__transitions_fired = []

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        parsed_msg = literal_eval(msg.data)
        self.__transitions_fired.append(parsed_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_node_publisher = gspn_executor()

    rclpy.spin(robot_node_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_node_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
