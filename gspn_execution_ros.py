from concurrent.futures.thread import ThreadPoolExecutor
from . import policy
from . import gspn as pn
import os
import sys
import numpy as np

from . import client

import rclpy

'''
__token_states is a list with the states of each token ['Free', 'Occupied', 'Done'] means that token 1 is Free, token 2
is Occupied and token 3 is Done.
__token_positions is a list with the places where each token is ['p1', 'p2', 'p2'] means that token 1 is on p1, token 2
is on p2 and token 3 is on p2.
'''


class GSPNexecutionROS(object):

    def __init__(self, gspn, place_to_function_mapping, output_to_transition_mapping, policy, project_path,
                 place_to_action_server_mapping):
        '''
        :param gspn: a previously created gspn
        :param place_to_function_mapping: dictionary where key is the place and the value is the function
        :param output_to_transition_mapping: dictionary where key is the output and the value is the transition
        :param policy: Policy object
        :param project_path: string with project path
        :param place_to_action_server_mapping: dictionary where key is the name of the place and the value
        is the name of the corresponding action server

        token_states is a list with len==number of tokens with Strings that represent the state of each token
        modules is a list with references to the imported functions that are used in the place to function mapping
        '''
        self.__gspn = gspn
        self.__token_states = []
        self.__token_positions = []

        self.__place_to_function_mapping = place_to_function_mapping
        self.__output_to_transition_mapping = output_to_transition_mapping

        self.__place_to_action_server_mapping = place_to_action_server_mapping

        self.__policy = policy
        self.__project_path = project_path

        self.__number_of_tokens = 0
        self.__action_clients = []

        self.__ac_list = []
        self.__client_node = 0

    def get_token_states(self):
        return self.__token_states

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

    def translate_arcs_to_marking(self, arcs, marking):
        translation = {}
        for i in arcs[0]:
            place = self.__gspn.index_to_places[i]
            translation[place] = 1
        return translation

    def fire_execution(self, transition, token_id):
        '''
        Fires the selected transition.
        :param transition: string with transition that should be fired
        :param token_id: int with the number of the token that is being fired
        '''
        arcs = self.__gspn.get_connected_arcs(transition, 'transition')
        index = self.__gspn.transitions_to_index[transition]
        marking = self.__gspn.get_current_marking()

        # 1 to 1
        if len(arcs[0]) == 1 and len(arcs[1][index]) == 1:
            new_place = self.__gspn.index_to_places[arcs[1][index][0]]
            self.__token_positions[token_id] = new_place
            self.__gspn.fire_transition(transition)

        # 1 to many
        elif len(arcs[0]) == 1 and len(arcs[1][index]) > 1:
            i = 0
            for i in range(len(arcs[1][index])):
                if i == 0:
                    new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                    self.__token_positions[token_id] = new_place
                else:
                    new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                    self.__token_positions.append(new_place)
                    self.__token_states.append('Free')
                    self.__number_of_tokens = self.__number_of_tokens + 1
                    self.__action_clients.append(self.__number_of_tokens)
            self.__gspn.fire_transition(transition)

        # many to 1
        elif len(arcs[0]) > 1 and len(arcs[1][index]) == 1:
            translation_marking = self.translate_arcs_to_marking(arcs, marking)
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
                        if self.__token_states[pos_index] == 'Waiting':
                            number_of_waiting = number_of_waiting + 1
                            break
            if number_of_waiting == len(translation_marking) - 1:
                check_flag = True
            else:
                check_flag = False

            if check_flag:
                new_place = self.__gspn.index_to_places[arcs[1][index][0]]
                old_place = self.__token_positions[token_id]
                self.__token_positions[token_id] = new_place
                self.__gspn.fire_transition(transition)
                for place_index in arcs[0]:
                    place_with_token_to_delete = self.__gspn.index_to_places[place_index]
                    if place_with_token_to_delete != old_place:
                        for j in range(len(self.__token_positions)):
                            if place_with_token_to_delete == self.__token_positions[j]:
                                index_to_del = j
                                self.__token_positions[index_to_del] = "null"
                                self.__token_states[index_to_del] = "VOID"
                                break
            else:
                self.__token_states[token_id] = 'Waiting'

        # many to many
        elif len(arcs[0]) > 1 and len(arcs[1][index]) > 1:
            translation_marking = self.translate_arcs_to_marking(arcs, marking)
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
                        self.__token_positions[token_id] = new_place
                    else:
                        new_place = self.__gspn.index_to_places[arcs[1][index][i]]
                        self.__token_positions.append(new_place)
                        self.__token_states.append('Free')
                        self.__number_of_tokens = self.__number_of_tokens + 1
                        self.__action_clients.append(self.__number_of_tokens)
                        self.__gspn.fire_transition(transition)

                # Delete tokens from previous places
                for place_index in arcs[0]:
                    place_with_token_to_delete = self.__gspn.index_to_places[place_index]
                    for j in range(len(self.__token_positions)):
                        if place_with_token_to_delete == self.__token_positions[j]:
                            index_to_del = j
                            self.__token_positions[index_to_del] = "null"
                            self.__token_states[index_to_del] = "VOID"
                            break
            else:
                self.__token_states[token_id] = 'Waiting'

    def apply_policy(self, token_id, result):
        '''
        Applies the calculated policy. If we have an immediate transition, the policy is checked. Otherwise, we simply
        fire the transition that resulted from the function that was executed.
        :param token_id: number of the token that is about to fire
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
                self.fire_execution(transition_to_fire, token_id)
            else:
                return -2

        # EXPONENTIAL transitions case
        else:
            self.fire_execution(result, token_id)
        print("AFTER", self.__gspn.get_current_marking())

    def decide_function_to_execute(self):
        '''
        Main execution cycle. At every instant, the threads check whether the tokens are done with their functions
        or not.
        max_workers = self.__number_of_tokens * 3 because of the case where we have new tokens being created.
        '''

        while True:
            number_tokens = len(self.__token_positions)
            for i in range(number_tokens):

                if self.__ac_list[i].goal_done():
                    print("I'm done")
                    print("result", self.__ac_list[i].get_result())


                    result = self.__ac_list[i].get_result()
                    print("apply policy", result)
                    res_policy = self.apply_policy(i, result)
                    print("return value of policy", res_policy)
                    self.__ac_list[i].set_done(False)
                    self.__ac_list[i].set_result(None)
                    self.__ac_list[i].set_free(True)

                else:
                    if self.__ac_list[i].client_free():

                        current_place = self.__token_positions[i]
                        splitted_path = self.__place_to_function_mapping[current_place].split(".")
                        action_type = splitted_path[0]
                        action_name = splitted_path[1]
                        self.__ac_list[i] = client.MinimalActionClient(node=self.__client_node, server_name=action_name)
                        self.__ac_list[i].send_goal(5)
                        self.__ac_list[i].set_free(False)
                        print("sent goal")

                    else:
                        rclpy.spin_once(self.__client_node)
                        print("spinning")


                '''
                if self.__token_states[action_client_number] == 'Free':
                    client_node = rclpy.create_node('minimal_action_client_' + str(action_client_number))
                    place = self.__token_positions[action_client_number]
                    splitted_path = self.__place_to_function_mapping[place].split(".")
                    action_type = splitted_path[0]
                    action_name = splitted_path[1]

                    self.__token_states[action_client_number] = 'Occupied'
                    self.__action_clients[action_client_number] = client.MinimalActionClient(node=client_node, server_name=action_name)
                    self.__action_clients[action_client_number].send_goal()
                    rclpy.spin(client_node)


                if self.__token_states[action_client_number] == 'Occupied' and self.__action_clients[action_client_number].goal_done():
                    print("i am done")
                    self.__token_states[action_client_number] = 'Done'


                if self.__token_states[action_client_number] == 'Done':
                    print("result", self.__action_clients[action_client_number].get_result())
                    res_policy = self.apply_policy(action_client_number, self.__action_clients[action_client_number].get_result())

                    if self.__token_states[action_client_number] == 'Waiting':
                        print("i am waiting")

                    elif res_policy == -2:
                        # This state is achieved when the token reaches a place with no connections.
                        self.__token_states[action_client_number] = 'Inactive'
                    else:
                        self.__token_states[action_client_number] = 'Free'
                    rclpy.init()
                    print("--------")
                '''

    def setup_execution(self):
        '''
        Prepares the following elements of the execution:
        1- token_states list and number of (initial) tokens;
        2- token_positions list;
        3- project path.
        '''
        # Setup project path
        path_name = self.get_path()
        self.__project_path = os.path.join(path_name)
        sys.path.append(self.__project_path)

        # Setup token_states list and number of (initial) tokens
        self.__number_of_tokens = self.__gspn.get_number_of_tokens()
        i = 0
        while i < self.__number_of_tokens:
            self.__token_states.append('Free')
            self.__action_clients.append(i)
            i = i + 1

        # Setup token_positions list
        marking = self.__gspn.get_current_marking()
        for place in marking:
            j = 0
            while j != marking[place]:
                self.__token_positions.append(place)
                j = j + 1

        #Setup action servers
        ## TODO:

        # Setup action clients
        rclpy.init()
        self.__client_node = rclpy.create_node('minimal_action_client')
        marking = self.__gspn.get_places()
        for place in marking:
            splitted_path = self.__place_to_function_mapping[place].split(".")
            action_type = splitted_path[0]
            server_name = splitted_path[1]
            value = marking[place]
            i = 0
            while i < value:
                action_client = client.MinimalActionClient(node=self.__client_node, server_name=server_name)
                self.__ac_list.append(action_client)
                i = i + 1



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

    if test_case == "a":
        my_pn = pn.GSPN()
        places = my_pn.add_places(['p1'], [1])
        trans = my_pn.add_transitions(['t1'], ['exp'], [1])
        arc_in = {}
        arc_out = {}
        a, b = my_pn.add_arcs(arc_in, arc_out)
        # Since I'm not using imm transitions, this part is irrelevant
        places_tup = ('p1')
        policy_dict = {(0, 1): {'t3': 0.5, 't4': 0.5}}
        policy = policy.Policy(places_tup, policy_dict)
        project_path = "/home/pedroac/ros2_ws/src"
        p_to_c_mapping = {'p1': 'Fibonacci.fibonacci_1'}
        p_to_as = {'p1': 'simple_action_server.py'}

        my_execution = GSPNexecutionROS(my_pn, p_to_c_mapping, True, policy, project_path,
                                     p_to_as)
        my_execution.setup_execution()
        my_execution.decide_function_to_execute()

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
        p_to_as = {'p1': 'simple_action_server.py', 'p2': 'fibonacci_action_server.py'}

        my_execution = GSPNexecutionROS(my_pn, p_to_c_mapping, True, policy, project_path,
                                     p_to_as)
        my_execution.setup_execution()
        my_execution.decide_function_to_execute()

    elif test_case == "c":
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
        p_to_f_mapping = {'p1': 'p1_action_function.main', 'p2': 'p2_action_function.main',
                          'p3': 'fibonacci_action_function.main', 'p4': 'p4_action_function.main',
                          'p5': 'p5_action_function.main', 'p6': 'p6_action_function.main',
                          'p7': 'p7_action_function.main', 'p8': 'p8_action_function.main',
                          'p9': 'fibonacci_action_function.main', 'p10': 'p10_action_function.main'}
        p_to_as = {'p1': 'simple_action_server.py', 'p2': 'simple_action_server.py',
                   'p3': 'fibonacci_action_server.py', 'p4': 'simple_action_server.py',
                   'p5': 'simple_action_server.py', 'p6': 'simple_action_server.py',
                   'p7': 'simple_action_server.py', 'p8': 'simple_action_server.py',
                   'p9': 'fibonacci_action_server.py', 'p10': 'simple_action_server.py'}

        my_execution = GSPNexecutionROS(my_pn, p_to_f_mapping, True, policy, project_path,
                                     p_to_as)
        my_execution.setup_execution()
        my_execution.decide_function_to_execute()


    else:
        print("Sorry, that test is not available yet. Try again in a few months!")

if __name__ == "__main__":
    main()
