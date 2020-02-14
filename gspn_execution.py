import gspn as pn
import gspn_tools as tools

'''
__places_with_token_list is essentially a dictionary where the key is the name of the place and the value is a list
of tokens that are in that place. This value can be randomly assigned by the user or not.
__initial_marking_dictionary has the same structure of __places_with_token_list and is only used to know how the initial 
marking looks like.
__token_states is a list with the states of each token ['FREE', 'OCCUPIED', 'DONE'] means that token 1 is Free, token 2
is Occupied and token 3 is Done. 
'''


class GSPNexecution(object):

    def __init__(self, gspn, function_place_mapping, output_transition_mapping, policy, path_file, places_with_token_list=False):
        '''
        :param gspn: a previously created gspn
        :param places_with_token_list: dictionary where the key is the name of the place and the value is a list
                                       of tokens that are in that place.
        '''
        self.__gspn = gspn
        self.__places_with_token_list = places_with_token_list
        self.__token_states = []

        self.__function_place_mapping = function_place_mapping
        self.__output_transition_mapping = output_transition_mapping

        self.__policy = policy
        self.__path_file = path_file

    def get_places_with_token_list(self):
        return self.__places_with_token_list

    def get_token_states(self):
        return self.__token_states

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
        else:
            return True

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

my_execution = GSPNexecution(my_pn, True, True, True, True)
my_execution.setup_execution()
print("tokens", my_execution.get_token_states())

