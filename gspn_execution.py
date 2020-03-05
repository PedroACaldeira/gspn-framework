from concurrent.futures.thread import ThreadPoolExecutor
import gspn as pn
import gspn_tools as tools
import os
import sys
import policy

'''
__token_states is a list with the states of each token ['Free', 'Occupied', 'Done'] means that token 1 is Free, token 2
is Occupied and token 3 is Done. 
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
        arcs = self.__gspn.get_connected_arcs(transition, 'transition')

        # On this case, we only switch the old place for the new
        if len(arcs[1][0]) == 1:
            new_place = self.__gspn.index_to_places[arcs[1][0][0]]
            self.__token_positions[token_id] = new_place

        # On this case, we have more than 1 transition firing, so we need to add elements
        else:
            i = 0
            for i in range(len(arcs[1][0])):
                if i == 0:
                    new_place = self.__gspn.index_to_places[arcs[1][0][i]]
                    self.__token_positions[token_id] = new_place
                else:
                    new_place = self.__gspn.index_to_places[arcs[1][0][i]]
                    self.__token_positions.append(new_place)
                    self.__token_states.append('Free')
                    self.__number_of_tokens = self.__number_of_tokens + 1
                    self.__futures.append(self.__number_of_tokens)

    def apply_policy(self, token_id):
        policy = self.get_policy()
        current_marking = self.__gspn.get_current_marking()
        order = policy.get_places_tuple()
        marking_tuple = self.convert_to_tuple(current_marking, order)
        pol_dict = policy.get_policy_dictionary()
        transition_dictionary = self.get_transitions(marking_tuple, pol_dict)
        if transition_dictionary != False:
            for transition in transition_dictionary:
                self.__gspn.fire_transition(transition)
                self.fire_execution(transition, token_id)
        else:
            return

    def decide_function_to_execute(self):
        with ThreadPoolExecutor(max_workers=self.__number_of_tokens) as executor:
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

                    if self.__futures[thread_number].done():
                        print(thread_number + 1, "Finished the task")
                        self.__token_states[thread_number] = 'Done'

                    if self.__token_states[thread_number] == 'Done':
                        self.apply_policy(thread_number)
                        self.__token_states[thread_number] = 'Free'
                        print("--------")


    def setup_execution(self):

        # Setup token_states list
        number_of_tokens = self.__gspn.get_number_of_tokens()
        i = 0
        while i < number_of_tokens:
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
        print(self.__project_path)

        # Setup number of (initial) tokens
        self.__number_of_tokens = len(self.__token_states)

    @staticmethod
    def make_executable(gspn):
        '''
        Returns an executable version of the Petri Net, where action places are unfolded in to two places
        :param gspn: (PN object)
        :return:    (PN object)
        '''
        all_places = gspn.get_current_marking()

        for place in all_places:
            exec_place = 'i.exec.' + place;
            exec_trans = 'ex.t.' + place;
            end_place = 'f.end_' + place;

            sub_places = {}
            sub_places = {exec_place: 0, end_place: 0}

            sub_trans = {}
            sub_trans[exec_trans] = ['imm', 1]

            arcs_in = {}
            arcs_in[exec_place] = [exec_trans]
            arcs_out = {}
            arcs_out[exec_trans] = [end_place]

            subPN = pn.GSPN()
            subPN.add_places_dict(sub_places)
            subPN.add_transitions_dict(sub_trans)
            subPN.add_arcs(arcs_in, arcs_out)

            gspn = tools.GSPNtools.expand_pn(gspn, subPN, place)

        return gspn


'''
my_pn = pn.GSPN()
places = my_pn.add_places(['p1', 'p2', 'p3', 'p4'], [1, 0, 0, 0])
trans = my_pn.add_transitions(['t1', 't2'], ['exp', 'exp'], [1, 1])
arc_in = {}
arc_in['p1'] = ['t1']
arc_in['p2'] = ['t2']
arc_out = {}
arc_out['t1'] = ['p2', 'p3']
arc_out['t2'] = ['p4']
a, b = my_pn.add_arcs(arc_in, arc_out)

places_tup = ('p1', 'p2', 'p3', 'p4')
policy_dict = {(1, 0, 0, 0): {'t1': 1}, (0, 1, 0, 0): {'t2': 1}}
policy = policy.Policy(places_tup, policy_dict)

project_path = "C:/Users/calde/Desktop/ROBOT"
p_to_f_mapping = {'p1': 'functions2.make_list', 'p2': 'folder.functions.execute_nu', 'p3': 'folder.functions.count_Number', 'p4':'folder.functions.execute_nu'}

my_execution = GSPNexecution(my_pn, p_to_f_mapping, True, policy, project_path)
my_execution.setup_execution()
my_execution.decide_function_to_execute()
'''

my_pn = pn.GSPN()
places = my_pn.add_places(['p1', 'p2', 'p3'], [3, 1, 0])
trans = my_pn.add_transitions(['t1', 't2', 't3'], ['exp', 'exp', 'exp'], [1, 1, 0.5])
arc_in = {}
arc_in['p1'] = ['t1']
arc_in['p1'] = ['t2']
arc_in['p2'] = ['t3']
arc_out = {}
arc_out['t1'] = ['p2']
arc_out['t2'] = ['p3']
arc_out['t3'] = ['p3']
a, b = my_pn.add_arcs(arc_in, arc_out)

places_tup = ('p1', 'p2', 'p3')
policy_dict = {(3, 1, 0): {'t1': 1}, (2, 2, 0): {'t1': 1}, (1, 3, 0): {'t1': 1}}
policy = policy.Policy(places_tup, policy_dict)

project_path = "C:/Users/calde/Desktop/ROBOT"
p_to_f_mapping = {'p1': 'folder.functions.count_Number', 'p2': 'folder.functions.execute_nu', 'p3': 'functions2.make_list'}

my_execution = GSPNexecution(my_pn, p_to_f_mapping, True, policy, project_path)
my_execution.setup_execution()
my_execution.decide_function_to_execute()

'''
places = my_pn.add_places(['p1', 'p2', 'p3', 'p4', 'p5'], [3, 1, 0, 5, 2])
trans = my_pn.add_transitions(['t1', 't2', 't3', 't4'], ['exp', 'exp', 'exp', 'exp'], [1, 1, 0.5, 0.5])
arc_in = {}
arc_in['p1'] = ['t1']
arc_in['p2'] = ['t2']
arc_in['p3'] = ['t3']
arc_in['p4'] = ['t4']
arc_in['p5'] = ['t1', 't3']
arc_out = {}
arc_out['t1'] = ['p2']
arc_out['t2'] = ['p5', 'p1']
arc_out['t3'] = ['p4']
arc_out['t4'] = ['p3', 'p5']
a, b = my_pn.add_arcs(arc_in, arc_out)


p_to_f_mapping = {'p1': functions.count_Number, 'p2': functions.execute_nu, 'p3': functions.make_list, 'p4': functions.make_pol, 'p5': functions.execute_pol}
my_execution = GSPNexecution(my_pn, p_to_f_mapping, True, True, True)
my_execution.setup_execution()

my_execution.execute_plan()
print("tokens", my_execution.get_token_states())
'''