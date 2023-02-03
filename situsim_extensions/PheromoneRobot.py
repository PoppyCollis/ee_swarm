import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
from situsim_v1_2 import *

# a subclass of Robot which drops pheromones in its position every drop_interval units of simulation time (0.5 units by
# default)
class PheromoneRobot(Robot):

    # construct robot
    # drop_interval was 0.5 originally
    def __init__(self, x, y, controller,
                 pheromone_manager=None,
                 left_light_sources=[],
                 right_light_sources=[],
                 radius=1, theta=0,
                 left_sensor_angle=np.pi/4,
                 right_sensor_angle=-np.pi/4,
                 left_sensor_noisemaker=None,
                 right_sensor_noisemaker=None,
                 field_of_view=2*np.pi,
                 max_speed=1, inertia_coeff=0,
                 drop_interval=4,
                 left_motor_noisemaker=None,
                 right_motor_noisemaker=None,
                 left_motor_max_speed=2,
                 right_motor_max_speed=2,
                 left_motor_inertia=0,
                 right_motor_inertia=0,
                 left_motor_reversed=False,
                 right_motor_reversed=False):

        super().__init__(x=x, y=y,
                         controller=controller,
                         left_light_sources=left_light_sources,
                         right_light_sources=right_light_sources,
                         radius=radius, theta=theta,
                         left_sensor_angle=left_sensor_angle,
                         right_sensor_angle=right_sensor_angle,
                         left_sensor_noisemaker=left_sensor_noisemaker,
                         right_sensor_noisemaker=right_sensor_noisemaker,
                         field_of_view=field_of_view,
                         max_speed=max_speed,
                         inertia_coeff=inertia_coeff,
                         left_motor_noisemaker=left_motor_noisemaker,
                         right_motor_noisemaker=right_motor_noisemaker,
                         left_motor_max_speed=left_motor_max_speed,
                         right_motor_max_speed=right_motor_max_speed,
                         left_motor_inertia=left_motor_inertia,
                         right_motor_inertia=right_motor_inertia,
                         left_motor_reversed=left_motor_reversed,
                         right_motor_reversed=right_motor_reversed)

        self.drop_rate = drop_interval
        self.pheromone_manager = pheromone_manager  # the manager which new pheromones will be added to
        self.timer = 0  # used to determine when it is time to drop the next pheromone

    # step robot. this robot behaves exactly the same as Robot, except that is also drop pheromones
    def step(self, dt):
        super().step(dt)
        self.timer += dt  # increment timer
        if self.timer >= self.drop_rate:  # if timer has expired, then drop a pheromone and restart timer
            self.timer = 0
            self.pheromone_manager.add_pheromone_at(self.x, self.y)
