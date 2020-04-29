from concurrent.futures.thread import ThreadPoolExecutor
from . import policy
from . import gspn as pn
import os
import sys
import numpy as np
from . import client
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import gspn_executor

class GSPNExecutionROS(object):

    def __init__(self, gspn, place_to_client_mapping, output_to_transition_mapping, policy, project_path, initial_place, robot_id):
        '''
        :param gspn: a previously created gspn
        :param place_to_client_mapping: dictionary where key is the place and the value is the function
        :param output_to_transition_mapping: dictionary where key is the output and the value is the transition
        :param policy: Policy object
        :param project_path: string with project path
        :param initial_place: string with the name of the robot's initial place

        self.__token_positions is a list with the places where each token is ['p1', 'p2', 'p2'] means that token 1 is on p1, token 2
        is on p2 and token 3 is on p2;
        self.__number_of_tokens is the current number of tokens;
        self.__action_clients is a list with all the action clients;
        self.__client_node is the node where the clients will be connected to.



        self.__current_place indicates where the robot is
        self.__action_client is the action client of the robot
        '''
        self.__gspn = gspn
        self.__token_positions = []

        self.__place_to_client_mapping = place_to_client_mapping
        self.__output_to_transition_mapping = output_to_transition_mapping

        self.__policy = policy
        self.__project_path = project_path

        self.__number_of_tokens = 0
        # should be removed after correcting many to many/many to 1/1 to many
        self.__action_clients = []
        # --
        self.__client_node = 0
        self.__client_node_subscriber = 0

        self.__current_place = initial_place
        self.__action_client = 0
        self.__robot_id = robot_id

    def get_path(self):
        return self.__project_path

    def get_policy(self):
        return self.__policy

    def convert_to_tuple(self, marking, order):
        '''
        :param marking: dictionary with key= places; value= number of tokens in place
        :param order: tuple of strings with the order of the marking on the policy
        :return: tuple with the marking
        '''
        marking_list = []
        for element in order:
            for key in marking:
                if element == key:
                    marking_list.append(marking[key])
        return tuple(marking_list)

    def get_transitions(self, marking, policy_dictionary):
        '''
        :param marking: tuple with the current marking (should already be ordered)
        :param policy_dictionary: dictionary where key=Transition name; value=probability of transition
        :return: transition dictionary if marking is in policy_dictionary; False otherwise
        '''
        for mark in policy_dictionary:
            if marking == mark:
                return policy_dictionary[mark]
        return False

    def translate_arcs_to_marking(self, arcs):
        '''
        : param arcs: set of connected add_arcs
        : return: dictionary where key is the place and the value is always 1
        This function essentially is used to determine which places are connected to the next transition.
        The purpose of this is to later on compare it with the current marking.
        '''
        translation = {}
        for i in arcs[0]:
            place = self.__gspn.index_to_places[i]
            translation[place] = 1
        return translation

    def fire_execution(self, transition):
        '''
        Fires the selected transition.
        :param transition: string with transition that should be fired
        '''
        arcs = self.__gspn.get_connected_arcs(transition, 'transition')
        index = self.__gspn.transitions_to_index[transition]
        marking = self.__gspn.get_current_marking()

        # 1 to 1
        if len(arcs[0]) == 1 and len(arcs[1][index]) == 1:
            new_place = self.__gspn.index_to_places[arcs[1][index][0]]
            self.__gspn.fire_transition(transition)
            self.__current_place = new_place

        # 1 to many
        elif len(arcs[0]) == 1 and len(arcs[1][index]) > 1:
            i = 0
            for i in range(len(arcs[1][index])):
                if i == 0:
                    new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                    # self.__token_positions[token_id] = new_place
                else:
                    new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                    self.__token_positions.append(new_place)
                    self.__number_of_tokens = self.__number_of_tokens + 1
                    self.__action_clients.append(client.MinimalActionClient("provisional", node=self.__client_node, server_name="provisional"))
            self.__gspn.fire_transition(transition)

        # many to 1
        elif len(arcs[0]) > 1 and len(arcs[1][index]) == 1:
            translation_marking = self.translate_arcs_to_marking(arcs)
            check_flag = True

            # We go through the marking and check it
            for el in translation_marking:
                if marking[el] < translation_marking[el]:
                    check_flag = False

            # We go through the states and see if all of them are 'Waiting'
            number_of_waiting = 0
            for place in translation_marking:
                for pos_index in range(len(self.__token_positions)):
                    if self.__token_positions[pos_index] == place:
                        if self.__action_clients[pos_index].get_state() == 'Waiting':
                            number_of_waiting = number_of_waiting + 1
                            break
            # -1 porque só precisas de ter x-1 a espera. quando o numero x
            # aparece, podes disparar
            if number_of_waiting == len(translation_marking) - 1:
                check_flag = True
            else:
                check_flag = False

            if check_flag:
                new_place = self.__gspn.index_to_places[arcs[1][index][0]]
                # old_place = self.__token_positions[token_id]
                # self.__token_positions[token_id] = new_place
                self.__gspn.fire_transition(transition)
                for place_index in arcs[0]:
                    place_with_token_to_delete = self.__gspn.index_to_places[place_index]
                    if place_with_token_to_delete != old_place:
                        for j in range(len(self.__token_positions)):
                            if place_with_token_to_delete == self.__token_positions[j]:
                                index_to_del = j
                                self.__token_positions[index_to_del] = "null"
                                self.__action_clients[index_to_del].set_state("VOID")
                                break
            else:
                print("many to one")
                # self.__action_clients[token_id].set_state("Waiting")

        # many to many
        elif len(arcs[0]) > 1 and len(arcs[1][index]) > 1:
            translation_marking = self.translate_arcs_to_marking(arcs)
            check_flag = True
            for el in translation_marking:
                if marking[el] < translation_marking[el]:
                    check_flag = False
            if check_flag:
                # Create tokens on next places
                i = 0
                for i in range(len(arcs[1][index])):
                    if i == 0:
                        new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                        # self.__token_positions[token_id] = new_place
                    else:
                        new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                        self.__token_positions.append(new_place)
                        self.__number_of_tokens = self.__number_of_tokens + 1
                        self.__action_clients.append(client.MinimalActionClient("provisional", node=self.__client_node, server_name="provisional"))
                        self.__gspn.fire_transition(transition)

                # Delete tokens from previous places
                for place_index in arcs[0]:
                    place_with_token_to_delete = self.__gspn.index_to_places[place_index]
                    for j in range(len(self.__token_positions)):
                        if place_with_token_to_delete == self.__token_positions[j]:
                            index_to_del = j
                            self.__token_positions[index_to_del] = "null"
                            self.__action_clients[index_to_del].set_state("VOID")
                            break
            else:
                print("many to many")
                # self.__action_clients[token_id].set_state("Waiting")

    def apply_policy(self, result):
        '''
        Applies the calculated policy. If we have an immediate transition, the policy is checked. Otherwise, we simply
        fire the transition that resulted from the function that was executed.
        :param result: result of the current place function
        :return: -2 if the current place has no output transitions. If successful, there is no return value
        '''

        print("BEFORE", self.__gspn.get_current_marking())
        # IMMEDIATE transitions case
        if result is None:
            execution_policy = self.get_policy()
            current_marking = self.__gspn.get_current_marking()
            order = execution_policy.get_places_tuple()
            marking_tuple = self.convert_to_tuple(current_marking, order)
            pol_dict = execution_policy.get_policy_dictionary()
            transition_dictionary = self.get_transitions(marking_tuple, pol_dict)
            if transition_dictionary:
                transition_list = []
                probability_list = []
                for transition in transition_dictionary:
                    transition_list.append(transition)
                    probability_list.append(transition_dictionary[transition])
                transition_to_fire = np.random.choice(transition_list, 1, False, probability_list)[0]
                print("TRANSITION TO FIRE", transition_to_fire)
                self.fire_execution(transition_to_fire)
            else:
                return -2

        # EXPONENTIAL transitions case
        else:
            self.fire_execution(result)
        print("AFTER", self.__gspn.get_current_marking())

    def ros_gspn_execution(self):
        '''
        Setup of the execution:
        1- project path;
        2- number of initial tokens;
        3- token_positions list;
        4- action servers;
        5- initial action clients.
        '''
        # Setup project path
        path_name = self.get_path()
        self.__project_path = os.path.join(path_name)
        sys.path.append(self.__project_path)

        # Setup number of (initial) tokens
        self.__number_of_tokens = self.__gspn.get_number_of_tokens()

        # Setup token_positions list
        marking = self.__gspn.get_current_marking()
        for place in marking:
            j = 0
            while j != marking[place]:
                self.__token_positions.append(place)
                j = j + 1

        #Setup action servers
        ## TODO:

        # Setup action client, publisher and subscriber
        rclpy.init()
        self.__client_node = gspn_executor.gspn_executor()
        #self.__client_node.subscription = self.__client_node.create_subscription(String, '/GSPN_MARKING', self.__client_node.listener_callback, 10)
        #self.__client_node.subscription

        splitted_path = self.__place_to_client_mapping[self.__current_place].split(".")
        action_type = splitted_path[0]
        server_name = splitted_path[1]
        self.__action_client = client.MinimalActionClient(action_type, node=self.__client_node, server_name=server_name)


        '''
        Main execution cycle. The execution is done step by step, meaning that there is no real paralelism.
        At each moment, we check whether the action client goal is complete and if not, we run spin_once()
        '''

        while True:
            if self.__action_client.get_state() == "Free":
                current_place = self.__current_place
                print("current place is", current_place)
                splitted_path = self.__place_to_client_mapping[current_place].split(".")
                # vais ter que meter aqui o ROS_DOMAIN para distinguir entre servers.
                action_type = splitted_path[0]
                server_name = splitted_path[1]
                self.__action_client = client.MinimalActionClient(action_type, node=self.__client_node, server_name=server_name)
                self.__action_client.send_goal(5)
                self.__action_client.set_state("Occupied")
                print("sent goal")

            if self.__action_client.get_state() == "Occupied":
                rclpy.spin_once(self.__client_node)

            if self.__action_client.get_state() == "Done":
                result = self.__action_client.get_result()
                res_policy = self.apply_policy(result)
                self.__client_node.talker_callback(result, self.__robot_id)

                rclpy.spin_once(self.__client_node)
                rclpy.spin_once(self.__client_node)

                # Update current marking phase
                received_data = self.__client_node.get_transitions_fired()
                print("received data", received_data)
                for i in range(len(received_data)):
                    if received_data[i][0] == None or received_data[i][1] == self.__robot_id:
                        continue
                    else:
                        self.fire_execution(received_data[i][0])
                        print("FIRED! ", self.__gspn.get_current_marking())
                self.__client_node.reset_transitions_fired()

                if self.__action_client.get_state() == "Waiting":
                    print("I am waiting")

                elif res_policy == -2:
                    print("The node doesn't have any output arcs.")
                    self.__action_client.set_state("Inactive")
                    self.__action_client.set_result(None)
                else:
                    self.__action_client.set_state("Free")
                    self.__action_client.set_result(None)
                print("------------")


            # check if marking has changed and if so, update it.

def main():

    from concurrent.futures.thread import ThreadPoolExecutor
    from . import policy
    from . import gspn as pn
    import os
    import numpy as np
    import rclpy
    from rclpy.node import Node
    import sys

    test_case = input("Enter case number to test: ")

    if test_case == "1":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2'], [1, 1])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {'p1': ['t1']}
        arc_out = {'t1': ['p2']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path)
        my_execution.ros_gspn_execution()

    elif test_case == "2":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2', 'p3'], [1, 1, 0])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {'p1': ['t1']}
        arc_out = {'t1': ['p2', 'p3']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2', 'p3':'Fibonacci.fibonacci_3'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path)
        my_execution.ros_gspn_execution()

    elif test_case == "3":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2', 'p3'], [1, 1, 0])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {'p1': ['t1'], 'p2':['t1']}
        arc_out = {'t1': ['p3']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2', 'p3':'Fibonacci.fibonacci_3'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path)
        my_execution.ros_gspn_execution()

    elif test_case == "4":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2', 'p3', 'p4'], [2, 0, 0, 0])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {'p1': ['t1'], 'p2':['t1']}
        arc_out = {'t1': ['p3', 'p4']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2', 'p3':'Fibonacci.fibonacci_3', 'p4':'Fibonacci.fibonacci_4'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path)
        my_execution.ros_gspn_execution()

    elif test_case == "5":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9', 'p10'],
                                  [1, 0, 0, 0, 0, 1, 0, 1, 0, 0])
        trans = my_pn.add_transitions(['t1', 't2', 't3', 't4', 't5', 't6'],
                                      ['exp', 'exp', 'exp', 'exp', 'exp', 'exp'],
                                      [1, 1, 1, 1, 1, 1])
        arc_in = {'p1': ['t1'], 'p2': ['t2'], 'p4':['t3'], 'p5':['t3'], 'p6':['t3'], 'p7':['t4'],
                  'p8': ['t4'], 'p10': ['t5', 't6']}
        arc_out = {'t1': ['p2'], 't2':['p3', 'p4', 'p5'], 't3': ['p7'], 't4':['p10', 'p9'],
                   't5': ['p2'], 't6':['p1']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        # project_path = "C:/Users/calde/Desktop/ROBOT"
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2',
                          'p3': 'Fibonacci.fibonacci_3', 'p4': 'Fibonacci.fibonacci_4',
                          'p5': 'Fibonacci.fibonacci_5', 'p6': 'Fibonacci.fibonacci_6',
                          'p7': 'Fibonacci.fibonacci_7', 'p8': 'Fibonacci.fibonacci_8',
                          'p9': 'Fibonacci.fibonacci_9', 'p10': 'Fibonacci.fibonacci_10'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path)
        my_execution.ros_gspn_execution()


    elif test_case == "a":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2'], [1, 1])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {'p1': ['t1']}
        arc_out = {'t1': ['p2']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path, 'p1', 1)
        my_execution.ros_gspn_execution()

    elif test_case == "b":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1', 'p2'], [1, 1])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {'p1': ['t1']}
        arc_out = {'t1': ['p2']}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1', 'p2')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1', 'p2': 'Fibonacci.fibonacci_2'}

        my_execution = GSPNExecutionROS(my_pn, p_to_c_mapping, True, policy, project_path, 'p2', 2)
        my_execution.ros_gspn_execution()

    else:
        print("Sorry, that test is not available yet. Try again in a few months!")

if __name__ == "__main__":
    main()
