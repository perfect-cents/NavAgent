# pylint: disable=multiple-statements, fixme, line-too-long, missing docstring, invalid-name


class State(object):
    def __init__(self, id):
        self.id = id

class Model_State(State):
    """docstring for Model_State."""
    def __init__(self, id):
        super(Model_State, self).__init__(id)

class Machine(object):
    """ The Main State Machine Class"""

    def __init__(self, states=None, initial=None):
        self.states = {}#OrderedDict()
        self._current_state = None

        self.add_state('null_state')

        if initial is None:
            initial = 'null_state'
        self._initial = initial

        self.add_states(states)

        self._set_state(self._initial)

    ############################################################################
    #   state getter/setter/deleter/adder/switcher functions
    ############################################################################

    def get_state(self, state):
        if state not in self.states:
            raise ValueError("{} is not a registered state.".format(state))
        return self.states[state]

    def _set_state(self, state):
        if isinstance(state, str):
            state = self.get_state(state)
        self._current_state = state

    def add_state(self, *args, **kwargs):
        self.add_states(*args, **kwargs)

    def del_state(self, *args, **kwargs):
        self.del_states(*args, **kwargs)

    def add_states(self, states):
        states = self._list_it(states)
        for state in states:
            if isinstance(state, str):
                if state in self.states:
                    raise ValueError("{} is already registered.".format(state))
                state = State(state)
            self.states[state.id] = state

    def del_states(self, states):
        states = self._list_it(states)
        for state in states:
            if isinstance(state, str):
                self.get_state(state)
                del self.states[state]

    def switch(self, start, dest):
        def states_check(states):
            states = self._list_it(states)
            for state in states:
                if isinstance(state, str) and state in self.states:
                    if self._current_state.id == state:
                        return True
                else:
                    return False

        if dest not in self.states:
            raise ValueError("{} is not a valid destination state.".format(dest))
        elif start == '*':
            self._current_state = self.states[dest]
        elif states_check(start):
            self._current_state = self.states[dest]
        else:
            raise ValueError("{} is not a valid starting state set.".format(start))


    def replace_state(self, state_to_replace, replacement_state, *replacement_args):
        """ Note: replacement_state must be an instance """
        if issubclass(replacement_state, State):
            if state_to_replace not in self.states:
                raise ValueError("{} is not a registered state.".format(state_to_replace))
            elif state_to_replace == self._current_state.id:
                self.states[state_to_replace] = replacement_state(state_to_replace, *replacement_args)
                self._current_state = self.states[state_to_replace]
            else:
                self.states[state_to_replace] = replacement_state(state_to_replace, *replacement_args)
        else:
            raise ValueError("{} needs to be a subclass of State.".format(replacement_state))

    def _list_it(self, obj):
        if obj is None:
            return []
        elif isinstance(obj, (list, tuple)):
            return obj
        else:
            return [obj]


    @property
    def initial(self):
        return self._initial

    @property
    def state(self):
        return self._current_state
