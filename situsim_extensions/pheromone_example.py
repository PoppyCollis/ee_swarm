import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *
import pygame
import matplotlib.pyplot as plt
import time
from pheromones import *
from PheromoneRobot import *
from plots2 import *
from plots3 import *
# from disturbances import *

# A controller to make a robot travel along a sinusoidal path, if f=1
# Other frequencies may lead to more interesting robot trajectories
class WiggleController(Controller):

    def __init__(self, f=1, gain=0.2, left_noisemaker=None, right_noisemaker=None):
        # NOTE: THIS CALL TO SUPER MUST BE HERE
        super().__init__(left_noisemaker, right_noisemaker) #
        self.t = 0  # time variable
        self.f = f  # frequency of sinusoidal control signal for right motor
        self.gain = gain  # the gain parameter determines how fast the robot moves

    # this controller does not actually use the inputs from the sensors, as the robot follows a sinusoidal path, but it is convenient for all motor step functions to take the same parameters
    def step(self, inputs, dt):
        self.t += dt  # update time variable
        self.left_speed_command = self.gain # left motor speed is constant
        self.right_speed_command = self.gain * (np.sin(self.t * self.f) + 1) # right motor speed varies sinusoidally between 0 and 2 * gain

        return super().step(inputs, dt)


# A simple example of a controller which has an internal state, 'x'
# This is not a useful controller, but implements the logistic map to update the internal state, which can therefore vary chaotically
class LogisticMapController(Controller):

    # construct controller
    def __init__(self, r=3.7, interval=1, left_noisemaker=None, right_noisemaker=None):
        # NOTE: THIS CALL TO SUPER MUST BE HERE
        super().__init__(left_noisemaker, right_noisemaker) #
        self.t = 0 # time variable
        self.x = 0.5 # initial value of x must be non-zero
        self.r = r # bifurcation parameter. for some values, including 3.7, x will appear to vary chaotically, but you can't always tell from looking at a single time series - in principle, a time series which looks chaotic could in fact be repeating with a very long period
        self.last_t = 0 # used to keep track of the last time the internal state changed
        self.interval = interval

    # this controller does not actually use the inputs from the sensors, but it is convenient for all motor step functions to take the same parameters
    def step(self, inputs, dt):
        self.t += dt  # update time variable
        if self.t - self.last_t > self.interval: # if we change the state every time we call step, the robot's velocity will appear almost constant, so we can set an interval between updates, which here is 1 unit of time
            self.last_t = self.t # keep track of time state was changed
            self.x = self.r * self.x * (1 - self.x) # update state
        # return 0.5, self.x # return motor speeds - one is constant; the other is set according to the internal state 'x'

        # set left motor speed
        self.left_speed_command = 0.5
        # set right motor speed
        self.right_speed_command = self.x

        return super().step(inputs, dt)

# A subclass of Controller
# - of course, the solution is a straightforward Braitenberg vehicle
class LightSeekingController(Controller):

    # init controller with passed in noisemakers and control parameters
    def __init__(self, left_noisemaker=None, right_noisemaker=None, speed=1):
        # NOTE: THIS CALL TO SUPER MUST BE HERE
        super().__init__(left_noisemaker, right_noisemaker) # call Controller.__init__() to set up noisemakers
        self.speed = speed

    # step method.
    # - MODIFY TO CLOSE THE SENSORIMOTOR LOOP!
    def step(self, inputs, dt):

        # set left motor speed
        self.left_speed_command = inputs[1] * self.speed
        # set right motor speed
        self.right_speed_command = inputs[0] * self.speed

        return super().step(inputs, dt)


# set up the pygame window, if we are animating the simulation
def setup_pygame_window(screen_width):
    # initialise pygame and set parameters
    pygame.init()
    screen = pygame.display.set_mode([screen_width, screen_width])
    pygame.display.set_caption("SituSim: sensory inversion challenge")
    # scale factor and offsets for converting simulation coordinates to pygame animation display coordinates
    pygame_scale = 30
    pygame_x_offset = screen_width/2
    pygame_y_offset = screen_width/2

    return screen

# draw SituSim systems in pygame window
def pygame_drawsim(screen, systems, width, paused, delay):

    running = True

    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                paused = not paused
            elif event.key == pygame.K_UP:
                delay -= 1
            elif event.key == pygame.K_DOWN:
                delay += 1

    delay = np.max([delay, 0])

    time.sleep(delay/100)

    screen.fill('black')

    # initial scale factor and offsets for converting simulation coordinates
    # to pygame animation display coordinates
    pygame_x_offset = width/2
    pygame_y_offset = width/2

    # find extremes of system trajectories for resizing animation window
    max_xs = []
    max_ys = []
    for system in systems:
        if system.has_position:
            max_xs.append(max(np.abs(system.xs)))
            max_ys.append(max(np.abs(system.ys)))

    # reset scale according to where systems are and have been
    pygame_scale = width / (2 * max(max(max_xs), max(max_ys)) + 1)

    # draw all systems
    for system in systems:
        system.pygame_draw(screen, scale=pygame_scale, shiftx=pygame_x_offset, shifty=pygame_y_offset)

    # flip the pygame display
    screen.blit(pygame.transform.flip(screen, False, True), (0, 0))
    # update the pygame display
    pygame.display.update()

    return running, paused, delay

# get a noisemaker. used here for robot's motors, but could also be used for its
# sensors or controllers. this is only for convenience - for all except the
# BrownNoiseSource, it would be better to have more control over the parameters
def get_noisemaker(noise_type, noise_param):
    noisemaker = None
    if noise_param > 0:
        if noise_type == 'brown':
            noisemaker = BrownNoiseSource(noise_param)
        elif noise_type == 'white':
            noisemaker = WhiteNoiseSource(min_val=-noise_param, max_val=noise_param)
        elif noise_type == 'spike':
            noisemaker = SpikeNoiseSource(prob=0.05, pos_size=noise_param, neg_size=-noise_param)
    return noisemaker

# main function, to run simulation and generate plots
def run_simulation_once(screen_width,
                        controller,
                        animate=False,
                        field_of_view=0.9*np.pi,
                        left_sensor_angle=np.pi/4,
                        right_sensor_angle=-np.pi/4,
                        duration=60,
                        left_motor_noise=0.1,
                        right_motor_noise=0.1,
                        noise_type='brown'
                        ):

    # create pheromone manager
    phermanager = PheromoneManager(decay_rate=0.02)

    # get noisemakers for robot's motors
    left_motor_noisemaker = get_noisemaker(noise_type, left_motor_noise)
    right_motor_noisemaker = get_noisemaker(noise_type, right_motor_noise)

    # construct the robot
    robot = PheromoneRobot(x=-5, y=0, theta=np.pi/2,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=left_motor_noisemaker,
                  right_motor_noisemaker=right_motor_noisemaker
                  )

    # create a standard light-seeking Braitenberg vehicle with noisy sensors.
    # N.B. that the pheromone managers list of pheromones is passed to both of the robot's sensors
    robot2 = Robot(x=-8, y=0, theta=0,
                   controller = LightSeekingController(speed=0.5),
                   left_light_sources=phermanager.pheromones,
                   right_light_sources=phermanager.pheromones,
                   field_of_view=np.pi,
                   left_motor_inertia=0,
                   right_motor_inertia=0)

    # construct the robot
    robot3 = PheromoneRobot(x=-9, y=0, theta=-np.pi/2,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None
                  )

    # construct the robot
    robot4 = PheromoneRobot(x=-4, y=4, theta=np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=3
                  )

    # construct the robot
    robot5 = PheromoneRobot(x=-8, y=-2, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot6 = PheromoneRobot(x=-4, y=-3, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot7 = PheromoneRobot(x=4, y=-7, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot8 = PheromoneRobot(x=-9, y=-1, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot9 = PheromoneRobot(x=-3, y=-3, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot10 = PheromoneRobot(x=0, y=-2, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    ##

    # construct the robot
    robot11 = PheromoneRobot(x=-6, y=0, theta=np.pi/2,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=left_motor_noisemaker,
                  right_motor_noisemaker=right_motor_noisemaker
                  )

    # create a standard light-seeking Braitenberg vehicle with noisy sensors.
    # N.B. that the pheromone managers list of pheromones is passed to both of the robot's sensors
    robot12 = Robot(x=-8, y=5, theta=0,
                   controller = LightSeekingController(speed=0.5),
                   left_light_sources=phermanager.pheromones,
                   right_light_sources=phermanager.pheromones,
                   field_of_view=np.pi,
                   left_motor_inertia=0,
                   right_motor_inertia=0)

    # construct the robot
    robot13 = PheromoneRobot(x=-2, y=3, theta=-np.pi/2,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None
                  )

    # construct the robot
    robot14 = PheromoneRobot(x=-1, y=-1, theta=np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=3
                  )

    # construct the robot
    robot15 = PheromoneRobot(x=9, y=-10, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot16 = PheromoneRobot(x=7, y=8, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot17 = PheromoneRobot(x=5, y=6, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot18 = PheromoneRobot(x=3, y=4, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot19 = PheromoneRobot(x=1, y=2, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )

    # construct the robot
    robot20 = PheromoneRobot(x=-1, y=-2, theta=-3*np.pi/4,
                  controller=LightSeekingController(speed=0.5),
                  field_of_view=field_of_view,
                  pheromone_manager=phermanager,
                  left_light_sources=phermanager.pheromones,
                  right_light_sources=phermanager.pheromones,
                  left_sensor_angle=left_sensor_angle,
                  right_sensor_angle=right_sensor_angle,
                  left_motor_inertia=0,
                  right_motor_inertia=0,
                  left_motor_noisemaker=None,
                  right_motor_noisemaker=None,
                  drop_interval=6
                  )


    # create list of agents - even though we only have one here, I always code
    # using a list, as it makes it easy to add more agents
    # agents = [robot, robot2, robot3]
    agents = [robot, robot2, robot3, robot4, robot5, robot6, robot7, robot8, robot9, robot10, robot11, robot12, robot13, robot14, robot15, robot16, robot17, robot18, robot19, robot20]

    # only run pygame code if animating the simulation
    if animate:
        screen = setup_pygame_window(screen_width)

    # animation variables
    delay = 0 # can be used to slow animation down
    running = True # can be used to exit animation early
    paused = False # can be used to pause simulation/animation

    # prepare simulation time variables
    t = 0
    ts = [t]
    dt = 1
    # dt = 0.1
    # begin simulation main loop
    while t < duration and running:

        # only move simulation forwards in time if not paused
        if not paused:

            # step all pheromones
            phermanager.step(dt)

            # step all robots
            for agent in agents:
                agent.step(dt)

            # increment time variable and store in ts list for plotting later
            t += dt
            ts.append(t)

        # only run pygame code if animating the simulation
        if animate:
            running, paused, delay = pygame_drawsim(screen, phermanager.pheromones + agents, screen_width, paused, delay)
    # simulation has completed

    # only run pygame code if animating the simulation
    if animate:
        # Quit pygame.
        pygame.display.quit()
        pygame.quit()

    return ts, agents, phermanager.pheromones

# plot outputs for all robots
# - note: these are not all of the outputs which we can plot,
# but they are the ones which will always be available,
# and which we will probably look at the most
def do_plots(all_ts, agents, light_sources):

    # parameters for plots
    plt.rcParams["font.weight"] = "bold"
    font_size = 18

    plot_all_agents_trajectories(all_ts, agents, light_sources, draw_agents=False)
    plot_all_robots_motors(all_ts, agents)
    plot_all_robots_controllers(all_ts, agents)
    plot_all_robots_sensors(all_ts, agents)
    plot_all_robots_motor_noise(all_ts, agents)
    # plot_all_robots_controller_noise(all_ts, agents)
    # plot_all_robots_sensor_noise(all_ts, agents)

    plt.show()

'''

select controller and parameters and run simulation

'''
def run_sim(runs=1, animate=True, left_motor_noise=0.1, right_motor_noise=0.1, noise_type='brown'):
    # set noise levels for controller outputs
    left_noise = 0.3
    right_noise = 0.4

    all_robots = []
    all_ts = []

    # run the simulation the specified number of times
    for i in range(runs):

        # NOTE: ALL SYSTEMS ARE CREATED INSIDE THIS LOOP, OR IN FUNCTIONS CALLED BY IT
        # - if we created a system outside of this loop, e.g. one of the noisemakers,
        # then it would be used in every run

        field_of_view = 0.9*np.pi
        left_sensor_angle = np.pi/4
        right_sensor_angle = -np.pi/4

        # create a controller object to pass to the robot
        controller = LogisticMapController()

        # if you uncomment this line, only the first run of the simulation will be animated
        animate = animate and i == 0

        # simulation duration
        duration = 500

        # run the simulation once, with the given parameters
        ts, robots, light_sources = run_simulation_once(screen_width=700,
                                                        controller=controller,
                                                        animate=animate,
                                                        field_of_view=field_of_view,
                                                        left_sensor_angle=left_sensor_angle,
                                                        right_sensor_angle=right_sensor_angle,
                                                        duration=duration,
                                                        left_motor_noise=left_motor_noise,
                                                        right_motor_noise=right_motor_noise,
                                                        noise_type=noise_type
                                                        )

        # build lists of robots and time values for plotting
        all_robots = all_robots + robots
        for _ in range(len(all_robots)):
            all_ts.append(ts)
    # plot data for all robots
    do_plots(all_ts, all_robots, light_sources)

# run the simulation
run_sim(runs=1, animate=True, left_motor_noise=0.9, right_motor_noise=0.7, noise_type='spike')
