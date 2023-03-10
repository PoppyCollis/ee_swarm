import sys
# add situsim locations to path so that imports work
sys.path.insert(1, '..')

# import situsim module and any extensions used
from situsim_v1_2 import *

# A model of a Homeostat unit. This is not a physically realistic model, and is
# as simple as possible while still having the right kind of dynamics.
class Unit(System):

    # construct a Unit
    def __init__(self, lower_viability, upper_viability, wait_time,
                 lower_limit=-np.Inf, upper_limit=np.Inf, theta0=0, theta_dot0=0):
        super().__init__() # call System constructor
        self.upper_viability = upper_viability # whenever the Unit variable is outside the limits of viability, it will adapt
        self.lower_viability = lower_viability
        self.upper_limit = upper_limit # these model physical hard limits to how far the Unit variable (needle) can actually move
        self.lower_limit = lower_limit
        self.theta = [theta0] # the system variable. theta is used instead of x, because in SituSim x is always used to mean a spatial coordinate
        self.theta_dot = [theta_dot0] # the system state is [theta, theta_dot]
        self.theta_dotdot = [0] # the input to the system is converted into an acceleration (technically, it is a force, but mass is not modelled here)
        self.units = [] # a list of connected Units
        self.weights = [] # a list of weights, which are applied to connected units
        self.adapting = False # this boolean variable keeps track of whether the system is in the process of adaptation
        self.was_adapting = [self.adapting] # a record of when the unit is adapting
        self.wait_time = wait_time # when the weights change, the Unit will wait for this period to see if the system stabilises
        self.been_waiting = 0 # how long the Unit has been waiting, if it is in the process of adapting
        self.weights_were = [] # a record of weights over time

    # step the system forwards in time by the interval dt
    def step(self, dt):

        # calculate weighted sum of inputs from all connected Units (including feedback from this Unit)
        input = 0
        for unit, weight in zip(self.units, self.weights):
            input += unit.get_theta() * weight

        # integrate the system
        theta_dotdot = (-self.theta_dot[-1] + input) # calculate acceleration
        theta_dot = (self.theta_dot[-1] + (self.theta_dotdot[-1] * dt)) # integrate acceleration to get velocity
        theta = (self.theta[-1] + (self.theta_dot[-1] * dt)) # integrate velocity to get position

        # in Ashby's Homeostat, there were hard limits to how far the needle
        # (system variable) could move in either direction - enforce these limits
        #   If you don't want to enforce limits, you can leave them at them
        # at the default values of +-np.Inf
        if theta > self.upper_limit:
            theta = self.upper_limit
            theta_dot = 0
        elif theta < self.lower_limit:
            theta = self.lower_limit
            theta_dot = 0

        # store system variable and its first and second derivatives
        self.theta.append(theta)
        self.theta_dot.append(theta_dot)
        self.theta_dotdot.append(theta_dotdot)

        # when weights are changed, you need to run the system for a while to
        # see if it stabilises or not - self.adapting is used to keep track of whether
        # this is process is currently ongoing
        if not self.adapting:
            # check to see if system (essential) variable is within specified viable limits
            #   - if it is not, then the Unit will adapt by changing its weights
            if not self.test_viability():
                self.adapt()
        else:
            # increment the time waiting since the weights were changed
            self.been_waiting += dt
            # if the Unit has been waiting for long enough, then reset the
            # adapting state and waiting time
            if self.been_waiting > self.wait_time:
                self.been_waiting = 0
                self.adapting = False

        # record the adapting state and weights
        self.was_adapting.append(self.adapting)
        self.weights_were.append(self.weights.copy())

        super().step(dt) # call System.step - not really needed here, but good practice
        return self.theta

    # test whether Unit variable is within specified limits
    def test_viability(self):

        # if the Unit is not already adapting and is not within viable limits,
        # then begin adaptation
        if ((self.theta[-1] > self.upper_viability) or
            (self.theta[-1] < self.lower_viability)):
            self.adapting = True
            return False # return False for not viable
        return True # return True for viable

    # adapt by changing system parameters
    def adapt(self):

        # adjust weights by a random amount in the interval [-0.5, 0.5]
        for i in range(len(self.weights)):
            self.weights[i] = np.random.random()-0.5

        # make the feedback connection negative
        # - Ashby didn't seem to do this, and it is not strictly necessary,
        #   but it will make the system recover stability much faster than when
        #   the feedback is allowed to be positive
        for i, h in enumerate(self.units):
            if h == self:
                self.weights[i] = -np.abs(self.weights[i])

    # connect a Unit to this Unit
    def add_connection(self, unit, weight):
        # add Unit to list
        self.units.append(unit)
        # add connection weight to weights list
        self.weights.append(weight)

    # get the state of the Unit variable (the full state of a Unit is
    # actually [theta, theta_dot], but other Units can only "see" theta)
    def get_theta(self):
        return self.theta[-1]

# A class which models Ashby's Homeostat, in a way which is as simple and
# abstract as possible. A Homeostat is constructed with the specified
# number of Units, which are *fully connected* to each other.
# - if you wish to implement other connection patterns, subclassing Homeostat and
# overriding connect_units is probably the easiest approach
class Homeostat(System):

    # construct a Homeostat
    def __init__(self, lower_viability, upper_viability, wait_time, unit_num=4,
                       lower_limit=-np.Inf, upper_limit=np.Inf):
        super().__init__() # call System constructor
        self.unit_num = unit_num # number of Units in the Homeostat
        self.units = [] # list of all Units in the Homeostat
        for _ in range(unit_num):
            self.units.append(Unit(lower_viability, upper_viability, wait_time, # add a new Unit to the Homeostat
                                   theta0=np.random.random()-0.5,
                                   theta_dot0=0,
                                   lower_limit = lower_limit,
                                   upper_limit = upper_limit))

        self.connect_units()

    # connect units. this method implements full connection, where all units
    # connect to all units, including themselves
    def connect_units(self):
        # connect all Units to each other, including themselves
        # - at this point, the feedback connections can be positive or negative
        for unit1 in self.units:
            for unit2 in self.units:
                unit1.add_connection(unit2, np.random.random()-0.5) # random weights in [-0.5, 0.5)

    # step the Homeostat. this only involves stepping the Units
    def step(self, dt):

        # step all Units
        for unit in self.units:
            unit.step(dt)

        super().step(dt) # call System.step - not really needed here, but good practice
