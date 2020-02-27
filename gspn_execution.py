from concurrent.futures.thread import ThreadPoolExecutor

import gspn as pn
import gspn_tools as tools
import logging
import threading
from pathlib import Path
import os
import sys
import time
import inspect
import importlib
from concurrent import futures

'''
__places_with_token_list is essentially a dictionary where the key is the name of the place and the value is a list
of tokens that are in that place. This value can be randomly assigned by the user or not.
__token_states is a list with the states of each token ['Free', 'Occupied', 'Done'] means that token 1 is Free, token 2
is Occupied and token 3 is Done. 
'''


class GSPNexecution(object):

    def __init__(self, gspn, place_to_function_mapping, output_to_transition_mapping, policy, project_path,
                 places_with_token_list=False):
        '''
        :param gspn: a previously created gspn
        :param place_to_function_mapping: dictionary where key is the place and the value is the function
        :param output_to_transition_mapping: dictionary where key is the output and the value is the transition
        :param policy:
        :param project_path: string with project path
        :param places_with_token_list: dictionary where the key is the name of the place and the value is a list
                                       of tokens that are in that place.

        token_states is a list with len==number of tokens with Strings that represent the state of each token
        modules is a list with references to the imported functions that are used in the place to function mapping
        '''
        self.__gspn = gspn
        self.__places_with_token_list = places_with_token_list
        self.__token_states = []

        self.__place_to_function_mapping = place_to_function_mapping
        self.__output_to_transition_mapping = output_to_transition_mapping

        self.__policy = policy
        self.__project_path = project_path

        self.__number_of_tokens = 0
        self.__set_of_threads = list()

    def get_places_with_token_list(self):
        return self.__places_with_token_list

    def get_token_states(self):
        return self.__token_states

    def get_path(self):
        return self.__project_path

    def get_policy(self):
        return self.__policy

    def apply_policy(self):
        policy = self.get_policy()
        print("applying policy")

    def look_for_token(self, id):
        '''
        :param id: int that represents the id of the desired token
        :return: string that represents the place where the token is. Returns false if error.
        '''
        for place in self.__places_with_token_list:
            for token in self.__places_with_token_list[place]:
                if token == id:
                    return place
        return False

    def decide_function_to_execute(self):
        with ThreadPoolExecutor(max_workers=self.__number_of_tokens) as executor:
            number_tokens = self.__gspn.get_number_of_tokens()
            for thread_number in range(number_tokens):
                if self.__token_states[thread_number] == 'Free':
                    place = self.look_for_token(thread_number + 1)
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
                    future = executor.submit(function_to_exec, thread_number)
                    '''
                    self.__set_of_threads[thread_number] = threading.Thread(target=function_to_exec())
                    self.__set_of_threads[thread_number].start()
                    self.__set_of_threads[thread_number].join()
                    '''
                    self.__token_states[thread_number] = 'Done'

                elif self.__token_states[thread_number] == 'Occupied':
                    print("I am occupied", thread_number + 1)
                    continue

                elif self.__token_states[thread_number] == 'Done':
                    print("I am done", thread_number + 1)
                    self.apply_policy()
                    self.__token_states[thread_number] = 'Free'

    def setup_execution(self):

        # Setup token_states list
        number_of_tokens = self.__gspn.get_number_of_tokens()
        i = 0
        while i < number_of_tokens:
            self.__token_states.append('Free')
            i = i + 1

        # Setup places_with_token_list dictionary randomly
        if self.__places_with_token_list == False:
            self.__places_with_token_list = {}
            places = self.__gspn.get_places()
            id_counter = 1
            for place in places:
                value = places[place]
                i = 0
                self.__places_with_token_list[place] = []
                while i < value:
                    self.__places_with_token_list[place].append(id_counter)
                    id_counter = id_counter + 1
                    i = i + 1
        '''
        WITH SIMPLE THREADS
        for index in range(len(self.__token_states)):
            print("Setup execution: create and start thread %d.", index)
            x = threading.Thread()
            self.__set_of_threads.append(x)
            x.start()
        '''
        # Setup project path
        path_name = self.get_path()
        self.__project_path = os.path.join(path_name)
        sys.path.append(self.__project_path)
        print(self.__project_path)

        # Setup number of (initial) tokens
        self.__number_of_tokens = len(self.__token_states)

    def execute_plan(self):
        # I should decide what the stopping condition will be.
        # Stopping condition idea: while there are enabled transitions
        counter = 0
        while counter != 2:
            self.decide_function_to_execute()
            counter = counter + 1

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

project_path = "C:/Users/calde/Desktop/ROBOT"
p_to_f_mapping = {'p1': 'folder.functions.count_Number', 'p2': 'folder.functions.execute_nu', 'p3': 'functions2.make_list'}

my_execution = GSPNexecution(my_pn, p_to_f_mapping, True, True, project_path)
my_execution.setup_execution()
my_execution.execute_plan()
'''
basepath = Path(root)
for entry in basepath.iterdir():
    if entry.is_dir():
        print(entry.name)

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
print("dict", my_execution.get_places_with_token_list())

my_execution.execute_plan()
print("tokens", my_execution.get_token_states())
'''