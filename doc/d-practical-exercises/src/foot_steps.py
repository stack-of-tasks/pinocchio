class FootSteps(object):
    '''
    The class stores three functions of time: left, right and flying_foot.
    Each function is piecewise constant. For each function, the user can ask
    what is the value of this function at time t.

    The storage is composed of three lists for left, right and flying_foot, and a list for time.
    The list of times stores the time intervals, i.e. each element of the list is
    the start of a time interval. The first element of the list is 0.
    The value of the functions left, right, flying_foot one this time interval is stored at
    the same position is their respective list (i.e. value of left on interval
    [time[i], time[i+1]] is stored in left[i].

    The 4 lists are set up using function add_phase().
    The values of functions left, right, flying_foot can be accessed through the function
    get_phase_type(t), get_left_position(t), get_right_position(t).
    phase_type are 'left' (meaning left foot is flying, right foot is fixed), 'right' (ie the opposite)
    or 'none' (meaning no foot is flying, both are fixed on the ground).

    Additionnally, functions get_left_next_position(t),
    get_right_next_position(t) can be used to get the next position of the
    flying foot (in that case, additional work is needed to compute the
    position of flying foot at time t by interpolating get_left_position(t)
    and get_left_next_position(t).

    Functions get_phase_start(t), get_phase_duration(t) and get_phase_remaining(t)
    can be used to get the starting time, the duration and the remaining time of the
    current phase at time t.
    '''

    def __init__(self, right, left):
        '''
        The class is initiated from the initial positions of left and right feet.
        '''
        self.right = [right]
        self.left = [left]
        self.time = [0.]
        self.flying_foot = []

    def add_phase(self, duration, foot, position=None):
        '''
        Add a phase lasting <duration> where the flyhing foot <foot> (either 'left' or 'right')
        moves to <position> (being a vector or a SE3 placement).
        Alternatively, <foot> might be set to 'none' (i.e double support). In that case, <position>
        is not specified (or is set to None, default).
        '''
        assert(foot == 'left' or foot == 'right' or foot == 'none')
        self.time.append(self.time[-1] + duration)
        self.right.append(self.right[-1])
        self.left.append(self.left[-1])
        self.flying_foot.append(foot)
        if foot == 'left':
            self.left[-1] = position
        elif foot == 'right':
            self.right[-1] = position

    def get_index_from_time(self, t):
        '''Return the index i of the interval containing t, i.e. t in time[i], time[i+1] '''
        if t > self.time[-1]:
            return len(self.time) - 1
        return next(i for i, ti in enumerate(self.time) if ti > t) - 1

    def get_phase_type(self, t):
        i = self.get_index_from_time(t)
        return self.flying_foot[i]

    def is_double_from_left_to_right(self, t):
        '''
        Suppose that phase at time <t> is a double support phase.
        Return True if the previous phase is left and/or the next phase is right.
        '''
        assert(self.get_phase_type(t) == 'none')
        i = self.get_index_from_time(t)
        return self.flying_foot[i - 1] == 'left' if i > 0 else self.flying_foot[i + 1] == 'right'

    def get_left_position(self, t):
        return self.left[self.get_index_from_time(t)]

    def get_right_position(self, t):
        return self.right[self.get_index_from_time(t)]

    def get_left_next_position(self, t):
        i = self.get_index_from_time(t)
        i = i + 1 if i + 1 < len(self.time) else i
        return self.left[i]

    def get_right_next_position(self, t):
        i = self.get_index_from_time(t)
        i = i + 1 if i + 1 < len(self.time) else i
        return self.right[i]

    def get_phase_start(self, t):
        return self.time[self.get_index_from_time(t)]

    def get_phase_duration(self, t):
        i = self.get_index_from_time(t)
        return self.time[i + 1] - self.time[i]

    def get_phase_remaining(self, t):
        return self.time[self.get_index_from_time(t) + 1] - t
