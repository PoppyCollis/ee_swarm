import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
sys.path.insert(1, '../situsim_extensions')
from situsim_v1_2 import *

# a Robot class which adds a second pair of sensors
class EBA_Robot(Robot):

    # construct robot
    def __init__(self, x, y, controller,
                 left_light_sources=[], right_light_sources=[],
                 left2_light_sources=[], right2_light_sources=[],
                 radius=1, theta=0,
                 left_sensor_angle=np.pi/4, right_sensor_angle=-np.pi/4,
                 left2_sensor_angle=np.pi/3, right2_sensor_angle=-np.pi/3,
                 left_sensor_noisemaker=None, right_sensor_noisemaker=None,
                 left2_sensor_noisemaker=None, right2_sensor_noisemaker=None,
                 field_of_view=2*np.pi,
                 left_motor_noisemaker=None,
                 right_motor_noisemaker=None,
                 left_motor_max_speed=2,
                 right_motor_max_speed=2,
                 left_motor_inertia=0,
                 right_motor_inertia=0,
                 left_motor_reversed=False,
                 right_motor_reversed=False,
                 field_of_view2=2*np.pi
                 ):
         self.left2_sensor_angle = left2_sensor_angle
         self.right2_sensor_angle = right2_sensor_angle
         self.left2_light_sources = left2_light_sources
         self.right2_light_sources = right2_light_sources
         self.left2_sensor = LightSensor(light_sources=left2_light_sources, x=x, y=y, noisemaker=left2_sensor_noisemaker, field_of_view=field_of_view2) # construct 2nd left sensor. at this point, dummy positions are given for light sensors. they will be fixed when self.update_sensor_postions() is called
         self.right2_sensor = LightSensor(light_sources=right2_light_sources, x=x, y=y, noisemaker=right2_sensor_noisemaker, field_of_view=field_of_view2) # construct 2nd right sensor
         self.left2_sensor.color = 'darkgreen'  # set second sensor pair colours
         self.right2_sensor.color = 'darkgreen'
         super().__init__(x=x, y=y, controller=controller,
                          radius=radius, theta=theta,
                          left_light_sources=left_light_sources,
                          right_light_sources=right_light_sources,
                          left_sensor_angle=left_sensor_angle,
                          right_sensor_angle=right_sensor_angle,
                          left_sensor_noisemaker=left_sensor_noisemaker,
                          right_sensor_noisemaker=right_sensor_noisemaker,
                          field_of_view=field_of_view,
                          left_motor_noisemaker=left_motor_noisemaker,
                          right_motor_noisemaker=right_motor_noisemaker,
                          left_motor_max_speed=left_motor_max_speed,
                          right_motor_max_speed=right_motor_max_speed,
                          left_motor_inertia=left_motor_inertia,
                          right_motor_inertia=right_motor_inertia,
                          left_motor_reversed=left_motor_reversed,
                          right_motor_reversed=right_motor_reversed
                          )

    # this method updates the positions and orientations of the robot's sensors every time it (potentially) moves
    def update_sensor_postions(self):
        super().update_sensor_postions()  # call Robot update method to update positions and angles for first sensor pair
        # update positions and angles for second sensor pair
        self.left2_sensor.x = self.state[0] + (self.radius * np.cos(self.state[2] + self.left2_sensor_angle))
        self.left2_sensor.y = self.state[1] + (self.radius * np.sin(self.state[2] + self.left2_sensor_angle))
        self.left2_sensor.theta = self.thetas[-1] + self.left2_sensor_angle
        self.right2_sensor.x = self.state[0] + (self.radius * np.cos(self.state[2] + self.right2_sensor_angle))
        self.right2_sensor.y = self.state[1] + (self.radius * np.sin(self.state[2] + self.right2_sensor_angle))
        self.right2_sensor.theta = self.thetas[-1] + self.right2_sensor_angle

    # the EBA_Robot requires a controller which can process four inputs, rather
    # than the usual two for a Robot
    def control(self, dt):
        # step all four sensors
        left_sensor_activation = self.left_sensor.step(dt)
        right_sensor_activation = self.right_sensor.step(dt)
        left2_sensor_activation = self.left2_sensor.step(dt)
        right2_sensor_activation = self.right2_sensor.step(dt)

        # get motor speed commands from controller
        left_speed_command, right_speed_command = self.controller.step([left_sensor_activation, right_sensor_activation,
                                                        left2_sensor_activation, right2_sensor_activation],
                                                        dt)
        # return motor speed commands
        return left_speed_command, right_speed_command

    # draw robot in the specified matplotlib axes
    def draw(self, ax):
        # call draw from super to draw Robot
        super().draw(ax)

        # the code that follows is just for drawing the additional pair of sensors
        self.left2_sensor.draw(ax)
        self.right2_sensor.draw(ax)

        self.draw_fov(self.left2_sensor, ax)
        self.draw_fov(self.right2_sensor, ax)

    # draw robot in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        # call draw from super to draw Robot
        super().pygame_draw(screen, scale, shiftx, shifty)

        # the code that follows is just for drawing the additional pair of sensors
        self.left2_sensor.pygame_draw(screen, scale, shiftx, shifty)
        self.right2_sensor.pygame_draw(screen, scale, shiftx, shifty)

        self.pygame_draw_fov(self.left2_sensor, screen, scale, shiftx, shifty)
        self.pygame_draw_fov(self.right2_sensor, screen, scale, shiftx, shifty)
