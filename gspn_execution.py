from concurrent.futures.thread import ThreadPoolExecutor
import gspn as pn
import os
import sys
import policy
import numpy as np

'''
__token_states is a list with the states of each token ['Free', 'Occupied', 'Done'] means that token 1 is Free, token 2
is Occupied and token 3 is Done. 
__token_positions is a list with the places where each token is ['p1', 'p2', 'p2'] means that token 1 is on p1, token 2
is on p2 and token 3 is on p2. 
'''


class GSPNexecution(object):

    def __init__(self, gspn, place_to_function_mapping, output_to_transition_mapping, policy, project_path):
        '''
        :param gspn: a previously created gspn
        :param place_to_function_mapping: dictionary where key is the place and the value is the function
        :param output_to_transition_mapping: dictionary where key is the output and the value is the transition
        :param policy: Policy object
        :param project_path: string with project path

        token_states is a list with len==number of tokens with Strings that represent the state of each token
        modules is a list with references to the imported functions that are used in the place to function mapping
        '''
        self.__gspn = gspn
        self.__token_states = []
        self.__token_positions = []

        self.__place_to_function_mapping = place_to_function_mapping
        self.__output_to_transition_mapping = output_to_transition_mapping

        self.__policy = policy
        self.__project_path = project_path

        self.__number_of_tokens = 0
        self.__futures = []

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

    def fire_execution(self, transition, token_id):
        '''
        Fires the selected transition.
        :param transition: string with transition that should be fired
        :param token_id: int with the number of the token that is being fired
        '''
        arcs = self.__gspn.get_connected_arcs(transition, 'transition')
        index = self.__gspn.transitions_to_index[transition]

        # On this case, we only switch the old place for the new
        if len(arcs[1][index]) == 1:
            new_place = self.__gspn.index_to_places[arcs[1][index][0]]
            self.__token_positions[token_id] = new_place

        # On this case, we have more than 1 transition firing, so we need to add elements to
        # __token_states and __token_positions
        else:
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
                    self.__futures.append(self.__number_of_tokens)

    def apply_policy(self, token_id, result):
        '''
        Applies the calculated policy. If we have an immediate transition, the policy is checked. Otherwise, we simply
        fire the transition that resulted from the function that was executed.
        :param token_id: number of the token that is about to fire
        :param result: result of the current place function
        :return: -2 if the current place has no output transitions. If successful, there is no return value
        '''
        print("BEFORE", self.__token_positions, self.__gspn.get_current_marking())
        if result is None:
            policy = self.get_policy()
            current_marking = self.__gspn.get_current_marking()
            order = policy.get_places_tuple()
            marking_tuple = self.convert_to_tuple(current_marking, order)
            pol_dict = policy.get_policy_dictionary()
            transition_dictionary = self.get_transitions(marking_tuple, pol_dict)

            if transition_dictionary:
                print("immediate transition, I should apply policy")
                transition_list = []
                probability_list = []
                for transition in transition_dictionary:
                    transition_list.append(transition)
                    probability_list.append(transition_dictionary[transition])
                transition_to_fire = np.random.choice(transition_list, 1, False, probability_list)[0]
                self.fire_execution(transition_to_fire, token_id)
                self.__gspn.fire_transition(transition_to_fire)
            else:
                return -2

        else:
            print("exponential transition, I should fire the transition on result")
            self.fire_execution(result, token_id)
            self.__gspn.fire_transition(result)
        print("AFTER", self.__token_positions, self.__gspn.get_current_marking())

    def decide_function_to_execute(self):
        '''
        Main execution cycle. At every instant, the threads check whether the tokens are done with their functions
        or not.
        max_workers = self.__number_of_tokens * 3 because of the case where we have new tokens being created.
        '''
        with ThreadPoolExecutor(max_workers=self.__number_of_tokens * 3) as executor:
            while True:
                number_tokens = self.__gspn.get_number_of_tokens()
                for thread_number in range(number_tokens):
                    if self.__token_states[thread_number] == 'Free':
                        place = self.__token_positions[thread_number]
                        splitted_path = self.__place_to_function_mapping[place].split(".")

                        # On the first case we have path = FILE.FUNCTION
                        if len(splitted_path) <= 2:
                            function_location = splitted_path[0]
                            function_name = splitted_path[1]
                            module_to_exec = __import__(function_location)
                            function_to_exec = getattr(module_to_exec, function_name)

                        # On the second case we have path = FOLDER. ... . FILE.FUNCTION
                        else:
                            new_path = splitted_path[0]
                            for element in splitted_path[1:]:
                                if element != splitted_path[-1]:
                                    new_path = new_path + "." + element

                            function_location = new_path
                            function_name = splitted_path[-1]
                            module_to_exec = __import__(function_location, fromlist=[function_name])
                            function_to_exec = getattr(module_to_exec, function_name)

                        self.__token_states[thread_number] = 'Occupied'
                        self.__futures[thread_number] = executor.submit(function_to_exec, thread_number)

                    if self.__futures[thread_number].done() and self.__token_states[thread_number] == 'Occupied':
                        self.__token_states[thread_number] = 'Done'

                    if self.__token_states[thread_number] == 'Done':
                        res_policy = self.apply_policy(thread_number, self.__futures[thread_number].result())
                        if res_policy == -2:
                            # This state is achieved when the token reaches a place with no connections.
                            self.__token_states[thread_number] = 'Inactive'
                        else:
                            self.__token_states[thread_number] = 'Free'
                        print("--------")

    def setup_execution(self):
        '''
        Prepares the following elements of the execution:
        1- token_states list and number of (initial) tokens;
        2- token_positions list;
        3- project path.
        '''

        # Setup token_states list and number of (initial) tokens
        self.__number_of_tokens = self.__gspn.get_number_of_tokens()
        i = 0
        while i < self.__number_of_tokens:
            self.__token_states.append('Free')
            self.__futures.append(i)
            i = i + 1

        # Setup token_positions list
        marking = self.__gspn.get_current_marking()
        for place in marking:
            j = 0
            while j != marking[place]:
                self.__token_positions.append(place)
                j = j + 1

        # Setup project path
        path_name = self.get_path()
        self.__project_path = os.path.join(path_name)
        sys.path.append(self.__project_path)


