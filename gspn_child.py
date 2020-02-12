from gspn import GSPN

'''
Since gspn.py was programmed before knowing that each token would need an id and a status, we created this child class 
in order to allow the manipulation of the tokens without compromising the other modules' integrity.  
__places_with_token_list is essentially a dictionary where the key is the name of the place and the value is a list
of tokens that are in that place. 
__initial_marking_dictionary has the same structure of __places_with_token_list and is only used to know how the initial 
marking looks like.
__token_states is a list with the states of each token ['FREE', 'OCCUPIED', 'DONE'] means that token 1 is Free, token 2
is Occupied and token 3 is Done. 
'''


class GSPN_Child(GSPN):

    def __init__(self):
        super().__init__()
        self.__places_with_token_list = {}
        self.__initial_marking_dictionary = {}
        self.__token_states = []

    def get_places_with_token_list(self):
        return self.__places_with_token_list

    def get_initial_marking_dictionary(self):
        return self.__initial_marking_dictionary

    def get_token_states(self):
        return self.__token_states

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

            if set_initial_marking:
                self.__initial_marking_dictionary = self.__places_with_token_list.copy()

            return True
        else:
            return False

    def remove_tokens_child(self, place_name, ntokens, set_initial_marking=False):
        '''
        Removes tokens from the places in the place_name list.
        :param place_name: (list) with the input places names, from where the tokens should be removed.
        :param ntokens: (list list int) with the ids of the tokens to be removed (must have the same order as in the place_name list)
        :param set_initial_marking: (bool) if True the number of tokens removed will also be added to the initial
                                    marking, if False the initial marking remains unchanged.
        :return: (bool) True if successful, and False otherwise
        '''
        place_name_backup = place_name.copy()
        if len(place_name) == len(ntokens):

            # Execution of the Father's code
            number_tokens_on_each_place = []
            i = 0
            for i in range(len(ntokens)):
                number_tokens_on_each_place.append(len(ntokens[i]))
            super(GSPN_Child, self).remove_tokens(place_name, number_tokens_on_each_place, set_initial_marking)

            # Execution of the new code
            for iterator in range(len(place_name_backup)):
                for t in range(len(ntokens[iterator])):
                    self.__places_with_token_list[place_name_backup[iterator]].remove(ntokens[iterator][t])

            if set_initial_marking:
                self.__initial_marking_dictionary = self.__places_with_token_list.copy()

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

            if set_initial_marking:
                self.__initial_marking_dictionary = self.__places_with_token_list.copy()

            return True
        else:
            return False

    def remove_place_child(self, place):
        '''
        Method that removes PLACE from Petri Net, with corresponding connected input and output arcs
        :param (str) Name of the place to be removed
        :return: (dict)(dict) Dictionaries containing input and output arcs connected to the removed place
        '''

        # Execution of Father's code
        super(GSPN_Child, self).remove_place(place)

        # Execution of the new code
        if place in self.__places_with_token_list:
            del self.__places_with_token_list[place]



my_pn = GSPN_Child()

places = my_pn.add_places_child(['p1', 'p2', 'p3', 'p4', 'p5'], [[1, 2], [3], [], [], []])
print("Before adding new tokens ", places)
print('Marking before adding new tokens: ', my_pn.get_current_marking(), '\n')
my_pn.add_tokens_child(['p1', 'p2'], [[4, 5], [6]])
print("After adding new tokens: ", my_pn.get_places_with_token_list())
print('Marking after adding new tokens: ', my_pn.get_current_marking(), '\n')
my_pn.remove_tokens_child(['p1', 'p2'], [[2, 4], [6]])
print('After removing tokens ', my_pn.get_places_with_token_list())
print('Marking after removing new tokens: ', my_pn.get_current_marking(), '\n')

my_pn.remove_place_child('p1')
print('After removing place ', my_pn.get_places_with_token_list())
print('Marking after removing new tokens: ', my_pn.get_current_marking(), '\n')
