import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
from situsim_v1_2 import *
import math
from numpy.random import choice
import random

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

def sigmoid2(x):
    return 100 / (1 + (math.exp(20+ (-0.02*x))))

# the implementation of this sensor is a bit odd, but it is here for use by HungryRobots, which may want to adapt their
# behaviours depending on how close to death (hungry) they are. in general, an organism or human machine will not have
# direct knowledge of its energy level, and some kind of sensor(s) will need to be used to apprehend it.
#
# the reason i don't especially like the implementation of this is that the robot's energy level is a property of the
# robot, and the sensor detects that property for use in the robot's controller. technically, this results in a circular
# reference, which I would normally avoid, but for now it works as desired, so it will be fixed in later implementations
# but is fine for now.
#
# the advantage of implementing this as a Sensor is that we can attach different kinds of NoiseSources to it, to model
# various kinds of imperfect sensing
class RobotEnergySensor(Sensor):

    # construct energy sensor
    def __init__(self, robot, x, y, noisemaker=None):
        super().__init__(x=x, y=y)
        self.activation = 0  # sensor activation. this variable is updated in and returned from the step method. it is stored separately in case you want to access it multiple times between simulation steps, although that is unlikely to be necessary
        self.activations = [self.activation]  # for plotting and analysis, a sensor keeps a complete record of its activation over time
        self.noisemaker = noisemaker
        self.robot = robot

    # step sensor
    def step(self, dt):
        super().step(dt)
        self.activation = self.robot.energy  # get robot's energy level

        # add noise, if a noisemaker is implemented
        if self.noisemaker != None:
            self.activation += self.noisemaker.step(dt)

        self.activations.append(self.activation)  # store sensor activation

        return self.activation  # return sensor activation


# a subclass of Robot, which can sense and consume food and poison objects (Consumables), and which gains and loses
# energy in those two cases
# an energy level sensor is implemented, although it is not used. the reason for implementing this is:
#   the energy level might make a good choice for essential variable (if you want to use one), and it could be made one
#   of the inputs to a controller
#       however, for biological agents in particular but human-made ones too, it is not possible to have perfect
#       knowledge of internal states such as energy levels. therefore, it would be more interesting if the input to
#       the controller came from a sensor with some level and type of noise applied to it
class HungryRobot(Robot):
    # construct robot
    # todo i have changed decayrate2 from 0.01 and decayrate from 1; 0.05 and 0.01
    def __init__(self, id, x, y, controller,
                 left_food_sources,
                 right_food_sources,
                 left_poison_sources,
                 right_poison_sources,
                 consumables, swarm, radius=1, theta=0,
                 initial_energy=10000,
                 left_food_sensor_angle=np.pi/4,
                 right_food_sensor_angle=-np.pi/4,
                 left_food_noisemaker=None,
                 right_food_noisemaker=None,
                 food_field_of_view=2*np.pi,
                 left_poison_sensor_angle=np.pi/4,
                 right_poison_sensor_angle=-np.pi/4,
                 left_poison_noisemaker=None,
                 right_poison_noisemaker=None,
                 poison_field_of_view=2*np.pi,
                 decay_rate=-0.05, decay_rate2=0.01,
                 max_speed=1, energy_sensor_noisemaker=None,
                 left_motor_noisemaker=None,
                 right_motor_noisemaker=None,
                 left_motor_max_speed=2,
                 right_motor_max_speed=2,
                 left_motor_inertia=0,
                 right_motor_inertia=0,
                 left_motor_reversed=False,
                 right_motor_reversed=False
                 ):

        self.id = id
        self.left_poison_sensor_angle = left_poison_sensor_angle  # sensor orientations
        self.right_poison_sensor_angle = right_poison_sensor_angle
        self.left_poison_sensor = LightSensor(light_sources=left_poison_sources, x=x, y=y, noisemaker=left_poison_noisemaker,
                                       field_of_view=poison_field_of_view)  # construct left poison sensor. at this point, dummy positions are given for light sensors. they will be fixed when the super constructor is called
        self.right_poison_sensor = LightSensor(light_sources=right_poison_sources, x=x, y=y, noisemaker=right_poison_noisemaker,
                                        field_of_view=poison_field_of_view)  # construct right poison sensor
        self.energy_sensor = RobotEnergySensor(robot=self, x=x, y=y, noisemaker=energy_sensor_noisemaker)  # there is a circular reference between robot and sensor - not ideal, but seems okay here
        self.consumables = consumables  # a list of consumables which will affect this Robot
        self.swarm = swarm
        self.timer = 0
        self.broadcast = None # tuple of value and index of gene; automatically value index is out of range

        super().__init__(x=x, y=y,
                         controller=controller,
                         radius=radius, theta=theta,
                         left_light_sources=left_food_sources,
                         right_light_sources=right_food_sources,
                         left_sensor_angle=left_food_sensor_angle,
                         right_sensor_angle=right_food_sensor_angle,
                         left_sensor_noisemaker=left_food_noisemaker,
                         right_sensor_noisemaker=right_food_noisemaker,
                         field_of_view=food_field_of_view,
                         left_motor_noisemaker=left_motor_noisemaker,
                         right_motor_noisemaker=right_motor_noisemaker,
                         left_motor_max_speed=left_motor_max_speed,
                         right_motor_max_speed=right_motor_max_speed,
                         left_motor_inertia=left_motor_inertia,
                         right_motor_inertia=right_motor_inertia,
                         left_motor_reversed=left_motor_reversed,
                         right_motor_reversed=right_motor_reversed
                         )  # call Robot constructor. The LightSensors already implemented in Robot function as food sensors

        self.left_sensor.color = 'darkgreen'  # set food sensor colours. poison sensor colours are left at the default, which is red
        self.right_sensor.color = 'darkgreen'
        self.energy_sensor.color = 'yellow'  # set energy level sensor
        self.energy = initial_energy  # set initial energy level
        self.energies = [initial_energy]  # store energy level
        self.decay_rate = decay_rate  # rate at which energy decays when used by motors
        self.decay_rate2 = decay_rate2  # rate at which energy decays even if the motors are inactive

    def update_sensor_postions(self):
        super().update_sensor_postions()  # call Robot update method to update positions and angles for food sensors
        # update positions and angles for poison sensors
        self.left_poison_sensor.x = self.state[0] + (self.radius * np.cos(self.state[2] + self.left_poison_sensor_angle))
        self.left_poison_sensor.y = self.state[1] + (self.radius * np.sin(self.state[2] + self.left_poison_sensor_angle))
        self.left_poison_sensor.theta = self.thetas[-1] + self.left_poison_sensor_angle
        self.right_poison_sensor.x = self.state[0] + (self.radius * np.cos(self.state[2] + self.right_poison_sensor_angle))
        self.right_poison_sensor.y = self.state[1] + (self.radius * np.sin(self.state[2] + self.right_poison_sensor_angle))
        self.right_poison_sensor.theta = self.thetas[-1] + self.right_poison_sensor_angle
        # update position of energy sensor
        self.energy_sensor.x = self.state[0]
        self.energy_sensor.y = self.state[1]

    def get_broadcast(self):
        gene = random.randint(0, len(self.controller.weights))
        broadcast = self.controller.weights.copy()
        # mutate by some amount
        broadcast[gene] = broadcast[gene] + random.randint(-1, 1)
        return broadcast

    # step robot
    def step(self, dt):
        # print(self.timer)
        # print(self.controller.weights, self.id)
        super().step(dt)  # call Robot's step function. note that the control method (below) gets called from there

        for consumable in self.consumables:
            if np.linalg.norm([self.x - consumable.x, self.y - consumable.y]) < consumable.radius:
                quantity = consumable.consume()
                if consumable.real_type == Consumables.food:
                    self.energy += quantity
                elif consumable.real_type == Consumables.poison:
                    self.energy -= quantity
                #todo resets robot to random starting position
                self.state = np.array([random.randint(-10,10), random.randint(-10,10), random.randint(0,10)])
                self.update_sensor_postions()
                # print('Energy: ' + str(self.energy))
        self.energies.append(self.energy)

        # TODO: Gene transfer mechanism
        if self.timer > 0 and self.timer < 150:
            self.timer += 1
        elif self.timer == 150:
            self.timer = 0
        else:
            for agent in self.swarm:
                if np.linalg.norm([self.x - agent.x, self.y - agent.y]) < self.radius*2:
                    # get fitness based on energy level
                    fitness = sigmoid2(self.energies[-1])

                    # BROADCASTING
                    # given own fitness, chance of broadcasting
                    num = random.uniform(0, 100)
                    if num < fitness:
                        # broadcast the genes
                        self.broadcast = self.get_broadcast()
                    else:
                        # do not broadcast
                        self.broadcast = None

                    # RECEIVING
                    # given own fitness, chance of receiving
                    num2 = random.uniform(0, 100)
                    if num2 > fitness:
                        # receive genes
                        received = agent.broadcast
                        if received is not None:
                            self.controller.weights = received
                    else:
                        # resist reproduction attempts
                        pass
                    self.timer = self.timer + 1

    # this is separated from the step method as it is easier to override in any subclasses of Robot than step, which
    # should be the same for all Robots
    def control(self, dt):
        # update all sensor measurements
        left_food_activation = self.left_sensor.step(dt)
        right_food_activation = self.right_sensor.step(dt)
        left_poison_activation = self.left_poison_sensor.step(dt)
        right_poison_activation = self.right_poison_sensor.step(dt)
        energy_activation = self.energy_sensor.step(dt)

        # get motor speeds from controller
        left_speed, right_speed = self.controller.step([left_food_activation, right_food_activation,
                                                        left_poison_activation, right_poison_activation,
                                                        energy_activation], dt)

        # todo I have made it so the robot does not lose energy proportional to motor speeds
        # update energy. the faster the robot's wheels turn, the quicker it loses energy
        self.energy -= np.abs(left_speed) * dt * self.decay_rate
        self.energy -= np.abs(right_speed) * dt * self.decay_rate
        self.energy -= dt * self.decay_rate2  # some energy is lost even if the robot does not move

        # todo: do i have remove this feature?
        self.energy = max(self.energy, 0)  # prevent energy falling below zero
        # if energy is zero, stop the motors
        #   you could do something more interesting here, e.g. have speed, or maximum speed be reduced as energy declines
        if self.energy > 0:
            return left_speed, right_speed
        else:
            # print('energy is zero so robot has stopped here')
            return 0, 0

    # draw robot in the specified matplotlib axes
    def draw(self, ax):
        # call draw from super to draw Robot
        super().draw(ax)

        # the code that follows is just for drawing the additional pair of sensors
        self.left_poison_sensor.draw(ax)
        self.right_poison_sensor.draw(ax)

        self.draw_fov(self.left_poison_sensor, ax)
        self.draw_fov(self.right_poison_sensor, ax)

    # draw robot in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        # call draw from super to draw Robot
        super().pygame_draw(screen, scale, shiftx, shifty)

        # the code that follows is just for drawing the additional pair of sensors
        self.left_poison_sensor.pygame_draw(screen, scale, shiftx, shifty)
        self.right_poison_sensor.pygame_draw(screen, scale, shiftx, shifty)

        self.pygame_draw_fov(self.left_poison_sensor, screen, scale, shiftx, shifty)
        self.pygame_draw_fov(self.right_poison_sensor, screen, scale, shiftx, shifty)

# a simple controller for a HungryRobot. As simple as this is, it will tend to drive towards food and away from poison.
# it doesn't work perfectly - I have noticed that it will sometimes drive over a poison object if there is food directly
# behind it
class HungryController(Controller):

    # construct controller
    def __init__(self,gain=1,
                 left_noisemaker=None,
                 right_noisemaker=None): # keyword argument - default weight settings, made to be changed.
        super().__init__(left_noisemaker, right_noisemaker) # call Controller constructor
        self.gain = gain  # the gain parameter determines how fast the robot moves
        self.internal_time = 0
        self.weights = list(np.random.randint(-8,7,4)) # this constitutes the entire genotype, each weight is a single gene
        self.w1 = np.array(self.weights[:3]) # first 5 weights are for left motor node
        self.w2 = np.array(self.weights[3:]) # second 5 weights are for right motor node

    # some inputs are unused here, but really good and adaptive controllers would probably use them all.
    # the inputs being passed in to this method are:
    #   inputs[0] = left_food_activation
    #   inputs[1] = right_food_activation
    #   inputs[2] = left_poison_activation
    #   inputs[3] = right_poison_activation
    #   inputs[4] = energy_activation - not used in this basic controller, but a really good one certainly would pay attention to it!

    # todo could I use inputs[4] as the rate at which the robot broadcasts a mutated version of a gene in its genotype?

    def step(self, inputs, dt):

        self.sensor_f = int(inputs[0]>inputs[1])
        self.nn_inputs = np.array([self.sensor_f, 1])  # 1 is the bias term

        self.left_speed_command = sigmoid(np.dot(np.array(self.weights[:2]).T, self.nn_inputs))
        self.right_speed_command =  sigmoid(np.dot(np.array(self.weights[2:]).T, self.nn_inputs))

        return super().step(inputs, dt)

# using an enum probably makes less sense in Python than it does in some other languages, as Python allows you to do
# pretty much anything you like to a variable, at any time you like, but the general idea is to use an enum to define
# and stick to a finite and constant set of values
class Consumables(Enum):
    food = 0
    poison = 1
    water = 2

# A consumable object, which can be placed in the environment, can be food, water or poison, and can be detected by
# LightSensors as it has a LightSource attached.
class Consumable(System):
    # todo recovery time was 10...
    # construct consumable
    def __init__(self, x, y, radius=0.5, quantity=10, recovery_time=1, real_type=Consumables.food,
                 apparent_type=Consumables.food):
        super().__init__(x, y)  # call System constructor, to allow for the possibility that a Consumable will move
        self.stimulus = LightSource(x=x, y=y)  # construct LightSource
        self.quantity = quantity  # the quantity determines how much of an effect consuming the item has on a HungryRobot
        self.recovery_time = recovery_time  # when an item is consumed, it can reappear when the recovery time expires. if you don't want it to recover, then just make this time longer than your simulation duration
        self.depleted = False  # initially, a Consumable is not depleted. When it is consumed, it is depleted and will be invisible until (and if) it recovers
        self.time_since_consumed = 0  # used to track time to recover
        self.radius = radius  # this is the radius within which a HungryRobot will consume it
        self.apparent_type = apparent_type
        self.real_type = real_type  # conceptually, a consumable has a real and an apparent type. the apparent type is
                                    # what it "looks" like, but the real_type determines the effect it has
                                    # in the current implementation, apparent_type is unused, as it is the stimulus
                                    # which belongs to a Consumable which is passed to a Sensor - the Sensor simply
                                    # detects what it detects, and it is the business of the Controller to interpret it

    # step consumable. Consumables are stepped in order to implement recovery from depletion
    def step(self, dt):
        super().step(dt)  # call System step method, to allow for the possibility that a Consumable will move
        if self.depleted: # if the Consumable has been depleted, then wait for recovery_time to replenish and make it detectable again
            if self.time_since_consumed >= self.recovery_time:  # if consumable has reached recovery_time
                self.depleted = False  # replenish consumable
                self.stimulus.is_on = True  # make consumable detectable again
            else:
                self.time_since_consumed += dt  # increment time since consumable was depleted

    # when a HungryRobot passes within the Consumable's radius, it calls this method so that the resource is depleted
    def consume(self):
        if self.depleted:  # if already depleted, return zero
            return 0
        else:  # if not already depleted, return the quantity which determines how much of an effect will be had on the robot
            self.depleted = True  # set to depleted
            self.stimulus.is_on = False  # turn LightSource off, to make the Consumable invisible
            self.time_since_consumed = 0
            return self.quantity

    # draw consumable in the specified matplotlib axes
    def draw(self, ax):
        self.set_color()
        alpha = 1
        if self.depleted:
            alpha = 0.3
        ax.add_artist(mpatches.Circle((self.x, self.y), self.radius, color=self.color, alpha=alpha))
        ax.plot(self.x, self.y, 'k.')

    def set_color(self):
        if self.real_type == Consumables.food:
            self.color = 'darkgreen'
        elif self.real_type == Consumables.water:
            self.color = 'blue'
        elif self.real_type == Consumables.poison:
            self.color = 'darkred'

    # draw consumable in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        self.set_color()
        width = 0
        if self.depleted:
            width = 2
        pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color=self.color, width=width, radius=scale*self.radius)
