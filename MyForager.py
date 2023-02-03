import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *
from disturbances import DisturbanceSource
# from HungryRobot import *
# from HungryRobot_PGTA import *
from HungryRobot_probabilistic2 import *
from Arena import *
import random
import pygame
from numpy import save


# TODO ADD OTHER DISTURBANCES: SENSOR POSITIONS MOVE, NOISE TO SENSORS, MOVE FOOD SOURCES
# TODO TRY AN ADAPTIVE MUTATION RATE?


# generate randomly placed food and poison items (consumables)
def generate_food_and_poison(food_num, poison_num, scale=10):

    # lists for storing all consumables, and the lights which are attached to them
    # - the stimuli (which are LightSource objects) are stored separately,
    #   as they are all that the robot's sensors will detect
    # - the consumables, on the other hand, are also passed to the robot as they
    #   are what affects the robot's energy levels
    foods_and_poisons = []
    foods_stimuli = []  # list of LightSources attached to food consumables, used by the robot's food sensors
    poisons_stimuli = []  # list of LightSources attached to food consumables, used by the robot's poison sensors
    # generate food items
    for i in range(food_num):
        x = random_in_interval(-scale, scale)
        y = random_in_interval(-scale, scale)
        # x = 0
        # y = 4
        #todo: here I have changed the recovery time AND the quantity. These need to be changed back to EQUAL levels.
        food = Consumable(x, y, radius=1, recovery_time=1, quantity=20, real_type = Consumables.food)
        foods_and_poisons.append(food)
        foods_stimuli.append(food.stimulus)
    # generate poison items
    for i in range(poison_num):
        # x = random_in_interval(-scale, scale)
        # y = random_in_interval(-scale, scale)
        # x=0
        # y = -4
        poison = Consumable(x, y, radius=1, recovery_time=1, quantity=20,
                            real_type=Consumables.poison)
        foods_and_poisons.append(poison)
        poisons_stimuli.append(poison.stimulus)

    return foods_stimuli, poisons_stimuli, foods_and_poisons

# it is a good idea to generate a controller for every runs
#   - this is in case the controller incorporates any kind of memory or internal
#   state, in which case it should not be used for multiple runs of a simulation
#   (i should have made it possible to reset a Controller, which would simplify
#   things a little)
def generate_controllers(gain, runs):
    # create a fresh controller for every run of the simulation
    controllers = []
    # for _ in range(runs)
    number_of_robots = 8
    # todo this should be set to the number of agents
    for _ in range(number_of_robots):
        # controllers.append(HungryController(gain=gain, weights = list(np.random.randint(-8,7,6))))
        controllers.append(HungryController(gain=gain))
    return controllers

# a function to generate all disturbances for a set of simulation runs
#   - this is partly necessary due to an issue with the way i programmed
#   DisturbanceSources, which makes it impossible to use a DisturbanceSource
#   more than once (i should have made it possible to reset a DisturbanceSource,
#   which would simplify things a little)
# in this function, only a single disturbance is generated for each run, but it
# it is relatively easy to modify this function to include multiple kinds of disturbance
def generate_disturbances(runs, start_times=[50]):

    # create a list of disturbance lists - one list for every run of the simulation
    disturbances_lists = []
    # for every run of the simulation, create a list of disturbances
    for i in range(runs):
        # list of disturbances
        disturbances = []
        # for every disturbance, we need to copy the lists of disturb_times
        # - this is because every DisturbanceSource **must** have its own lists - they cannot share!
        start_times_ = []
        for s in start_times:
            start_times_.append(s)
        # add the motor direction disturbance
        # disturbances.append(NewFoodSwitcheroo(start_times=start_times_,
        #                                                     enabled=True))
        disturbances.append(FoodSwitcheroo(start_times=start_times_,
                                           enabled=True))



        # this is where you could potentially add more disturbances to the simulation
        # - for every disturbance you want to use, construct it and append it to disturbances

        # add the disturbance list (for a single simulation run) to the list of lists
        disturbances_lists.append(disturbances)

    return disturbances_lists

# this is for use with a HungryRobot
# - like the SensoryInversionDisturbanceSource, its effect is one-shot - every time it is triggered it fires and then is
#   immediately disabled.
# - the effect of a switcheroo is to turn poison into food and food into poison. this has no effect on any sensors
#   which detect those items, so when what the HungryRobot is attracted to as food starts to kill it, and poison starts
#   to feed it, it has to detect that and adapt its behaviour accordingly
#
# THESE LINES APPLY TO A FEATURE WHICH HAS BEEN DISABLED FOR THIS CHALLENGE, BY BEING COMMENTED OUT - IF YOU UNCOMMENT
# THE LINE IN THE STEP FUNCTION WHICH DISTURBS THE ROBOT, THE EFFECT WILL BE AS DESCRIBED BELOW
# the disturbance also changes the decay_rate parameter of the given robot, a change made specifically for this challenge. when
# the consumables are food, the decay_rate determines the energetic cost of movement. When the consumables are poison,
# penalising movement in the usual way may be an issue - if there is a cost to movement, and a cost to consuming poison,
# then the best strategy to minimise costs may be to simply stop moving (the robot will still lose some energy, even
# when not moving, but this loss happens in either case).
# when the sign of decay_rate is inverted, energy grows when the robot moves, i.e. movement is  directly rewarded,
# so that the best strategy for the robot to maximise energy is to keep moving and avoid poison at the same time.
class NewFoodSwitcheroo(DisturbanceSource):

    # construct disturbance source
    def __init__(self, start_times, enabled, robot=None, consumables=None):
        super().__init__(start_times, end_times=[], enabled=enabled)
        if consumables:
            self.add_consumables(consumables)
        if robot:
            self.add_robot(robot)

    # add robot to DisturbanceSource - this allows for the DisturbanceSource to be created in advance of the Robot,
    # so that the same pattern of using a function to generate DisturbanceSources in advance can be followed as was
    # used in the ealier motor_inversion_challenge.py file
    def add_robot(self, robot):
        self.robot = robot

    # add consumables to DisturbanceSource - this allows for the DisturbanceSource to be created in advance of the
    # consumables, so that the same pattern of using a function to generate DisturbanceSources in advance can be
    # followed as was used in the ealier motor_inversion_challenge.py file
    def add_consumables(self, consumables):
        self.consumables = consumables

    # step disturbance source
    def step(self, dt):
        super().step(dt)
        if self.enabled:
            print('distubance!')
            # self.robot.decay_rate = -self.robot.decay_rate  # switch between penalising and rewarding motion
            for consumable in self.consumables:  # switch consumable types
                if consumable.real_type == Consumables.food:
                    consumable.real_type = Consumables.poison
                elif consumable.real_type == Consumables.poison:
                    consumable.real_type = Consumables.food

            self.enabled = False  # this is a one-shot disturbance, so disable it again immediately


class FoodSwitcheroo(DisturbanceSource):

    # construct disturbance source
    def __init__(self, start_times, enabled, robots=None, consumables=None):
        super().__init__(start_times, end_times=[], enabled=enabled)
        if consumables:
            self.add_consumables(consumables)
        if robots:
            self.add_robot(robots)

    # add robot to DisturbanceSource - this allows for the DisturbanceSource to be created in advance of the Robot,
    # so that the same pattern of using a function to generate DisturbanceSources in advance can be followed as was
    # used in the ealier motor_inversion_challenge.py file
    def add_robot(self, robots):
        self.robots = robots

    # add consumables to DisturbanceSource - this allows for the DisturbanceSource to be created in advance of the
    # consumables, so that the same pattern of using a function to generate DisturbanceSources in advance can be
    # followed as was used in the ealier motor_inversion_challenge.py file
    def add_consumables(self, consumables):
        self.consumables = consumables

    # step disturbance source
    def step(self, dt):
        super().step(dt)
        if self.enabled:
            print("disturbance!")
            # self.robot.decay_rate = -self.robot.decay_rate  # switch between penalising and rewarding motion
            for consumable in self.consumables:  # switch consumable types
                if consumable.real_type == Consumables.food:
                    consumable.quantity = consumable.quantity * -1

            for robot in self.robots:
                robot.decay_rate = robot.decay_rate * -1
                robot.decay_rate2 = robot.decay_rate2 * -1
                print('decay', robot.decay_rate)
                print('decay2', robot.decay_rate2)

            self.enabled = False  # this is a one-shot disturbance, so disable it again immediately



# this function tests a disturbance which inverts the direction of either one or both of a robots motor directions
# at the specified times
def test_forager_controller(controllers, disturb=False, disturbances_lists=[],
                            sensor_noise=False, runs=1, animate=False):

    # params for plots: I had to make the font bold and bigger than usual for my fullscreen png saves
    # - (saving as eps or svg gave too large files)
    # plt.rcParams["font.weight"] = "bold"
    font_size = 18

    # # figure for motor speeds
    # fig2, axs2 = plt.subplots(2, 1)
    # # figure for motor speed commands
    # fig3, axs3 = plt.subplots(2, 1)
    # # figure for sensors
    # fig4, axs4 = plt.subplots(2, 1)
    #
    # # used to find min and max coordinate values used later for setting axis limits on robot trajectory plots
    # x_min = 1e6
    # x_max = -1e6
    # y_min = 1e6
    # y_max = -1e6

    # copy disturbance times for plotting later
    times = []
    for s in disturbances_lists[0][0].start_times:
        times.append(s)

    # lists used for averaging fitness scores
    fitnesses = []

    # run simulation 'runs' times
    for j in range(runs):

        # figure for robot trajectory
        # fig, axs = plt.subplots(1, 2, gridspec_kw={'width_ratios': [3.2, 4]})

        # get fresh controller for this simulation run
        controller=controllers[j]
        # get fresh list of disturbances for every simulation run
        disturbances = disturbances_lists[j]

        # set up noise sources for robot sensors
        left_food_noisemaker = None
        right_food_noisemaker = None
        left_poison_noisemaker = None
        right_poison_noisemaker = None
        energy_sensor_noisemaker = None
        if sensor_noise:
            left_food_noisemaker = WhiteNoiseSource(min_val=-0.2, max_val=0.2)
            right_food_noisemaker = WhiteNoiseSource(min_val=-0.2, max_val=0.2)
            left_poison_noisemaker = WhiteNoiseSource(min_val=-0.2, max_val=0.2)
            right_poison_noisemaker = WhiteNoiseSource(min_val=-0.2, max_val=0.2)
            energy_sensor_noisemaker = WhiteNoiseSource(min_val=-5, max_val=5)

        # generate randomly placed food and poison items
        # food_num = 15
        food_num = 5
        # poison_num = 15
        poison_num = 0
        scale = 10
        foods_stimuli, poisons_stimuli, foods_and_poisons = generate_food_and_poison(food_num, poison_num, scale)

        # create robot
        robot = HungryRobot(id = 1, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controller, left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot2 = HungryRobot(id = 2, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+1], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot3 = HungryRobot(id = 3, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+2], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot4 = HungryRobot(id = 4, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+3], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot5 = HungryRobot(id = 5, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+4], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot6 = HungryRobot(id = 6, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+5], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot7 = HungryRobot(id = 7, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+6], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # create robot
        robot8 = HungryRobot(id = 8, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j+7], left_food_sources=foods_stimuli,
                            right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
                            right_poison_sources=poisons_stimuli,
                            left_food_sensor_angle=np.pi/3,
                            right_food_sensor_angle=-np.pi/3,
                            left_poison_sensor_angle=np.pi/5,
                            right_poison_sensor_angle=-np.pi/5,
                            food_field_of_view=0.8*np.pi, poison_field_of_view=0.8*np.pi,
                            consumables=foods_and_poisons,
                            swarm = [],
                            energy_sensor_noisemaker=energy_sensor_noisemaker,
                            left_food_noisemaker=left_food_noisemaker,
                            right_food_noisemaker=right_food_noisemaker,
                            left_poison_noisemaker=left_poison_noisemaker,
                            right_poison_noisemaker=right_poison_noisemaker)

        # # create robot
        # robot9 = HungryRobot(id=9, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 8],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot10 = HungryRobot(id=10, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 9],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot11 = HungryRobot(id=11, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 10],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot12 = HungryRobot(id=12, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 11],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot13 = HungryRobot(id=13, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 12],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot14 = HungryRobot(id=14, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 13],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot15 = HungryRobot(id=15, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 14],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)
        #
        # # create robot
        # robot16 = HungryRobot(id=16, x=random.randint(-10, 10), y=random.randint(-10, 10), controller=controllers[j + 15],
        #                      left_food_sources=foods_stimuli,
        #                      right_food_sources=foods_stimuli, left_poison_sources=poisons_stimuli,
        #                      right_poison_sources=poisons_stimuli,
        #                      left_food_sensor_angle=np.pi / 3,
        #                      right_food_sensor_angle=-np.pi / 3,
        #                      left_poison_sensor_angle=np.pi / 5,
        #                      right_poison_sensor_angle=-np.pi / 5,
        #                      food_field_of_view=0.8 * np.pi, poison_field_of_view=0.8 * np.pi,
        #                      consumables=foods_and_poisons,
        #                      swarm=[],
        #                      energy_sensor_noisemaker=energy_sensor_noisemaker,
        #                      left_food_noisemaker=left_food_noisemaker,
        #                      right_food_noisemaker=right_food_noisemaker,
        #                      left_poison_noisemaker=left_poison_noisemaker,
        #                      right_poison_noisemaker=right_poison_noisemaker)

        # add robot to the Disturbance - not really needed in this challenge
        # disturbances[0].add_robot(robot)
        # add consumables to the Disturbance
        disturbances[0].add_consumables(foods_and_poisons)

        agents = [robot,robot2,robot3, robot4, robot5, robot6, robot7, robot8]
        # agents = [robot,robot2,robot3, robot4, robot5, robot6, robot7, robot8, robot9, robot10, robot11, robot12, robot13, robot14, robot15, robot16]
        # agents = [robot,robot2,robot3, robot4]

        disturbances[0].add_robot(agents)

        # this bit of code gives each agent a list of all other agents in the simulation (bar itself)
        # todo this is needed for the bump detector
        for i,agent in enumerate(agents):
            agent_list = agents.copy()
            agent_list.pop(i)
            agent.swarm.extend(agent_list)
        # for agent in agents:
        #     print(agent.swarm)

        # arena
        arena = Arena(agents, x_left=-11, x_right=11, y_top=11, y_bottom=-11)

        # only simulate using pygame in first run of simulation
        if animate:
            # initialise pygame and set parameters
            pygame.init()
            pygame.display.set_caption('SituSim animation')
            ############ SET THIS WIDTH TO FIT YOUR MONITOR RESOLUTION ############
            width = 700
            screen = pygame.display.set_mode([width, width])
            # scale factor and offsets for converting simulation coordinates to pygame animation display coordinates
            pygame_scale = 30
            pygame_x_offset = width/2
            pygame_y_offset = width/2

        # prepare simulation time variables                                                                                # the simulation
        t = 0
        ts = [t]
        dt = 0.1

        # main simulation loop
        running = True
        while running:

            if disturb:
                # step disturbances
                for disturbance in disturbances:
                    disturbance.step(dt)
            # step all consumables
            for consumable in foods_and_poisons:
                consumable.step(dt)
            # step all robots
            for agent in agents:
                agent.step(dt)

            arena.step(dt)

            # access the current fitness of each agent by calling the last value input of the energies list
            # print(robot.energies[-1], robot2.energies[-1])

            # increment time variable and store in ts list for plotting later
            t += dt
            ts.append(t)

            animate = animate and (j == 0) # only animate the first run, if any
            #duration
            running = running and (t < 5000) # run for a maximum of 300 units of time
            # The simulation runs until we close the pygame window
            if animate:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        running = False
                screen.fill('black')

                # draw all consumables
                for consumable in foods_and_poisons:
                    consumable.pygame_draw(screen, scale=pygame_scale, shiftx=pygame_x_offset, shifty=pygame_y_offset)
                # draw robot

                for r in agents:
                    r.pygame_draw(screen, scale=pygame_scale, shiftx=pygame_x_offset, shifty=pygame_y_offset)

                arena.pygame_draw(screen, scale=pygame_scale, shiftx=pygame_x_offset, shifty=pygame_y_offset)

                # flip the pygame display
                screen.blit(pygame.transform.flip(screen, False, True), (0, 0))
                # update the pygame display
                pygame.display.update()


        # Quit pygame.
        pygame.display.quit()
        pygame.quit()

        # calculate fitness
        fitness = np.mean(robot.energies)
        print("Run fitness: " + str(fitness))
        fitnesses.append(fitness)

        # TODO NORMALISE Y-AXIS!
        # plot actual and sensed energy levels
        # - these will be the same, unless you turn noise on
        plt.plot(ts, robot.energies, label='Robot 1')
        plt.plot(ts, robot2.energies, label='Robot 2')
        plt.plot(ts, robot3.energies, label='Robot 3')
        plt.plot(ts, robot4.energies, label='Robot 4')
        plt.plot(ts, robot5.energies, label='Robot 5')
        plt.plot(ts, robot6.energies, label='Robot 6')
        plt.plot(ts, robot7.energies, label='Robot 7')
        plt.plot(ts, robot8.energies, label='Robot 8')
        # plt.plot(ts, robot9.energies, label='Robot 9')
        # plt.plot(ts, robot10.energies, label='Robot 10')
        # plt.plot(ts, robot11.energies, label='Robot 11')
        # plt.plot(ts, robot12.energies, label='Robot 12')
        # plt.plot(ts, robot13.energies, label='Robot 13')
        # plt.plot(ts, robot14.energies, label='Robot 14')
        # plt.plot(ts, robot15.energies, label='Robot 15')
        # plt.plot(ts, robot16.energies, label='Robot 16')

        average = np.array((np.array(robot.energies) + np.array(robot2.energies) + np.array(robot3.energies) + np.array(robot4.energies) + np.array(robot5.energies) + np.array(robot6.energies) + np.array(robot7.energies) + np.array(robot8.energies))/8)
        # average = np.array((np.array(robot.energies) + np.array(robot2.energies) + np.array(robot3.energies) + np.array(robot4.energies) + np.array(robot5.energies) + np.array(robot6.energies) + np.array(robot7.energies) + np.array(robot8.energies)+ np.array(robot9.energies)+ np.array(robot10.energies) + np.array(robot11.energies) + np.array(robot12.energies)+ np.array(robot13.energies)+ np.array(robot14.energies)+ np.array(robot15.energies)+ np.array(robot16.energies) )/16)
        # average = np.array((np.array(robot.energies) + np.array(robot2.energies) + np.array(robot3.energies) + np.array(robot4.energies)))/4

        trial = 100

        save(f'retest2/trial_{trial}.npy', average)

        plt.plot(ts, average , color='limegreen', linewidth=5, label='Average')

        # plt.plot(ts, robot.energy_sensor.activations, label='detected energy')
        # plt.plot(ts, robot2.energy_sensor.activations, label='detected energy')
        # ax[0].xlabel('time')
        # ax[0].ylabel('energy')
        # ax[0].title('Energy over time')
        # plot times when disturbance takes effect
        # for s in times:
        #     plt.plot([s, s], [0, np.max(robot.energies)], 'r--', label='disturbance')
        #     plt.plot([s, s], [0, np.max(robot2.energies)], 'r--', label='disturbance')
        plt.ylim(ymax=10000, ymin=0)
        plt.title('Energy levels during single evolutionary run: disturbance')
        plt.xlabel('Time')
        plt.ylabel('Energy level')

        plt.legend()
        plt.show()

        # # find min and max robot coordinate values for setting axis limits later
        # if min(robot.xs) < x_min:
        #     x_min = min(robot.xs)
        # if max(robot.xs) > x_max:
        #     x_max = max(robot.xs)
        # if min(robot.ys) < y_min:
        #     y_min = min(robot.ys)
        # if max(robot.ys) > y_max:
        #     y_max = max(robot.ys)
        #
        # # params for plots: I had to make the font bold and bigger than usual for my fullscreen png saves
        # # - (saving as eps or svg gave too large files)
        # plt.rcParams["font.weight"] = "bold"
        # font_size = 18
        #
        # # draw all consumables
        # for consumable in foods_and_poisons:
        #     consumable.draw(axs[0])
        #     consumable.draw(axs[1])
        #
        # # set supertitle for trajectory plots window
        # fig.suptitle('Robot trajectories', fontsize=font_size, fontweight='bold')
        # # plot robot trajectory
        # axs[0].plot(robot.xs, robot.ys)
        # axs[0].plot(robot.xs[-1], robot.ys[-1], 'r*')
        # # fix plot axis proportions to equal
        # axs[0].set_aspect('equal')
        #
        # # # only draw colorbar once
        # show_bar = True
        # tcp.doColourVaryingPlot2d(robot.xs, robot.ys, ts, fig, axs[1], map='plasma', showBar=show_bar)
        # # make lines and text bold, for full-screen png save to get high resolution
        # if show_bar:
        #     cbar = axs[1].collections[-1].colorbar
        #     cbar.set_label(label='time', fontsize=font_size, weight='bold')
        #     cbar.outline.set_linewidth(3)
        #     cbar.ax.tick_params(direction='out', length=6, width=3)
        #     for t in cbar.ax.get_yticklabels():
        #         t.set_fontsize(font_size)
        #
        # # fix axis limits
        # axs[0].set_xlim([x_min-1, x_max+1])
        # axs[0].set_ylim([y_min-1, y_max+1])
        # # fix axis proportions to equal
        # axs[0].set_aspect('equal')
        #
        # # fix axis limits
        # axs[1].set_xlim([x_min-1, x_max+1])
        # axs[1].set_ylim([y_min-1, y_max+1])
        # # fix axis proportions to equal
        # axs[1].set_aspect('equal')
        #
        # # fix spacing between plots
        # fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        #
        # # plot motor speeds
        # axs2[0].plot(ts, robot.left_motor.speeds)
        # axs2[1].plot(ts, robot.right_motor.speeds)
        # for axis in axs2:
        #     axis.set_ylabel('Motor speed', fontsize=font_size, fontweight='bold')
        # axs2[0].set_title('Left speed', fontsize=font_size, fontweight='bold')
        # axs2[1].set_title('Right speed', fontsize=font_size, fontweight='bold')
        #
        # # fix spacing between plots
        # fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
        #
        # # plot motor speed commands
        # axs3[0].plot(ts, robot.controller.left_speed_commands)
        # axs3[1].plot(ts, robot.controller.right_speed_commands)
        # for axis in axs3:
        #     axis.set_ylabel('Motor speed commands', fontsize=font_size, fontweight='bold')
        # axs3[0].set_title('Left speed command', fontsize=font_size, fontweight='bold')
        # axs3[1].set_title('Right speed command', fontsize=font_size, fontweight='bold')
        #
        # # fix spacing between plots
        # fig3.tight_layout(rect=[0, 0.03, 1, 0.95])
        #
        # # plot sensor activations
        # axs4[0].plot(ts, robot.left_sensor.activations)
        # axs4[1].plot(ts, robot.right_sensor.activations)
        # for axis in axs4:
        #     axis.set_ylabel('Sensor activations', fontsize=font_size, fontweight='bold')
        # axs4[0].set_title('Left sensor', fontsize=font_size, fontweight='bold')
        # axs4[1].set_title('Right sensor', fontsize=font_size, fontweight='bold')
        #
        # # fix spacing between plots
        # fig4.tight_layout(rect=[0, 0.03, 1, 0.95])
        #
        # plt.figure()
        # # plot actual and sensed energy levels
        # # - these will be the same, unless you turn noise on
        # plt.xlabel('time', fontsize=font_size, fontweight='bold')
        # plt.ylabel('energy', fontsize=font_size, fontweight='bold')
        # plt.title('Energy over time', fontsize=font_size, fontweight='bold')
        # plt.plot(ts, robot.energy_sensor.activations, label='detected energy')
        # plt.plot(ts, robot2.energy_sensor.activations, label='detected energy')
        # plt.plot(ts, robot.energies, label='Actual energy')
        # plt.plot(ts, robot2.energies, label='Actual energy')
        # # plot times when disturbance takes effect
        # for s in times:
        #     plt.plot([s, s], [0, np.max(robot.energies)], 'r--', label='disturbance')
        #     plt.plot([s, s], [0, np.max(robot2.energies)], 'r--', label='disturbance')
        # plt.legend()

        # todo I think there is a problem plotting the graphs because there is only one singular energy sensor, given
        # that this is how it has been built. Check on creating individual energy sensors for each robot
        # BUT MAYBE NOT ????

        # make lines and text bold, for full-screen png save to get high resolution
        # for axes in [axs4, axs3, axs2]:
        #     for axis in axes:
        #         axis.set_xlabel('time', fontsize=font_size, fontweight='bold')
        #         axis.tick_params(axis='both', which='major', labelsize=font_size)
        #         axis.tick_params(axis='both', which='minor', labelsize=font_size)
        #         axis.tick_params(direction='out', length=6, width=3)
        #         for border in ['top', 'bottom', 'left', 'right']:
        #             axis.spines[border].set_linewidth(3)

    # return average of fitnesses
    return np.mean(fitnesses)

# specify number of simulation runs
runs = 1
# generate a list of controllers, so that every simulation run uses a fresh controller


controllers = generate_controllers(gain=1, runs=runs)

# generates lists of disturbances. every simulation run gets a list of disturbances (although here, we only have one disturbance in each list, we could have more)
disturbances_lists = generate_disturbances(start_times=[5000], runs=runs)

# run simulation for specified number of runs
avg_fitness = test_forager_controller(controllers=controllers, sensor_noise=False,
                                             disturb=False, disturbances_lists=disturbances_lists,
                                             runs=runs, animate=True)

# print quantitative results to terminal
print("Average fitness = " + str(avg_fitness))

# show all matplotlib plots
# plt.show()
