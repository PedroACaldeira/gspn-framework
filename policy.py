

class Policy(object):
    def __init__(self, places_tuple, policy_dictionary):
        '''
        :param places_tuple: tuple with the order of the places that are represented in policy_dictionary
        :param policy_dictionary: dictionary where key= tuple with the marking; value= dictionary where key is
        transitions and value is the probability of firing
        '''
        self.__places_tuple = places_tuple
        self.__policy_dictionary = policy_dictionary

    def get_places_tuple(self):
        return self.__places_tuple

    def get_policy_dictionary(self):
        return self.__policy_dictionary
