from gspn import GSPN
import token_class

'''
Since gspn.py was programmed before knowing that each token would need an id and a status, we created this child class 
in order to allow the manipulation of the tokens without compromising the other modules' integrity.  
__places_with_token_list is essentially a dictionary where the key is the name of the place and the value is a list
of tokens that are in that place. 
'''


class GSPN_Child(GSPN):

    def __init__(self):
        super().__init__()
        self.__places_with_token_list = {}
        self.__token_number = 0
        # list of strings with size = number of tokens, where each element is the state of the token
        self.__token_states = []

    def get_places_with_token_list(self):
        return self.__places_with_token_list

    def add_tokens_child(self, place_name, ntokens, set_initial_marking=False):
        '''
        Adds extra tokens to the places in the place_name list.
        :param place_name: (list) with the input places names, to where the tokens should be added
        :param ntokens: (list list int) with the number of tokens to be added (must have the same order as in the place_name list)
        :param set_initial_marking: (bool) if True the number of tokens added will also be added to the initial
                                    marking, if False the initial marking remains unchanged
        :return: (bool) True if successful, and False otherwise
        '''
        place_name_backup = place_name.copy()
        if len(place_name) == len(ntokens):

            # Execution of the Father's code
            number_tokens_on_each_place = []
            i = 0
            for i in range(len(ntokens)):
                number_tokens_on_each_place.append(len(ntokens[i]))
            super(GSPN_Child, self).add_tokens(place_name, number_tokens_on_each_place, set_initial_marking)

            # Execution of the new code
            for iterator in range(len(place_name_backup)):
                for t in range(len(ntokens[iterator])):
                    self.__places_with_token_list[place_name_backup[iterator]].append(ntokens[iterator][t])
            return True
        else:
            return False

    def add_places_child(self, name, ntokens, set_initial_marking=True):
        '''
        Adds new places to the existing ones in the GSPN object. Replaces the ones with the same name.
        :param name: (list str) denoting the name of the places
        :param ntokens: (list list token) denoting which tokens are in each place
        :param set_initial_marking: (bool) denoting whether we want to define ntokens as the initial marking or not
        '''
        if len(name) == len(ntokens):
            # Execution of the Father's code
            number_tokens_on_each_place = []
            i = 0
            for i in range(len(ntokens)):
                number_tokens_on_each_place.append(len(ntokens[i]))
            super(GSPN_Child, self).add_places(name, number_tokens_on_each_place, set_initial_marking)

            # Execution of the new code
            for iterator in range(len(name)):
                self.__places_with_token_list[name[iterator]] = []
                for t in range(len(ntokens[iterator])):
                    self.__places_with_token_list[name[iterator]].append(ntokens[iterator][t])

            return self.__places_with_token_list.copy()
        else:
            return False


my_pn = GSPN_Child()

places = my_pn.add_places_child(['p1', 'p2', 'p3', 'p4', 'p5'], [[1, 2], [3], [], [], []])
print("Before adding new tokens ", places)
print('Marking before adding new tokens: ', my_pn.get_current_marking(), '\n')
my_pn.add_tokens_child(['p1', 'p2'], [[4,5], [6]])
print("After adding new tokens: ", my_pn.get_places_with_token_list())
print('Marking after adding new tokens: ', my_pn.get_current_marking(), '\n')


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

a = my_pn.get_enabled_transitions()





