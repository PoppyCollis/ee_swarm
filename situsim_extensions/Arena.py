import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
from situsim_v1_2 import *

# a class to represent a rectangular arena, which will constrain the
# position of agents inside of it
# - this class relies on agents having a radius attribute
class Arena(System):

    # construct Arena with a list of agents to constrain and coordinates of walls
    def __init__(self, agents, x_left, x_right, y_top, y_bottom):
        # call System constructor
        super().__init__(x=0, y=0, theta=None)
        # set attributes
        self.agents = agents
        self.x_left = x_left
        self.x_right = x_right
        self.y_top = y_top
        self.y_bottom = y_bottom

    # step arena
    def step(self, dt, x_move=None, y_move=None):

        # if move parameters are passed to step, then shift the arena
        if x_move and y_move:
            self.move(x_move, y_move)
        # call step of System, so that new xy-coordinates are stored
        super().step(dt)

        # for all agents, constrain them to remain inside the box
        for agent in self.agents:
            # constrain in y
            if (agent.state[1] + agent.radius) > self.y_top:
                agent.state[1] = self.y_top - agent.radius
            elif (agent.state[1] - agent.radius) < self.y_bottom:
                agent.state[1] = self.y_bottom + agent.radius
            # constrain in x
            if (agent.state[0] + agent.radius) > self.x_right:
                agent.state[0] = self.x_right - agent.radius
            elif (agent.state[0] - agent.radius) < self.x_left:
                agent.state[0] = self.x_left + agent.radius

    # move (translate) arena by specifed increments
    def move(self, x_move, y_move):
        self.x += x_move
        self.y += y_move
        self.x_left += x_move
        self.x_right += x_move
        self.y_top += y_move
        self.y_bottom += y_move

    # draw arena in the specified matplotlib axes
    def draw(self, ax):
        ax.plot([self.x_left, self.x_left,
                 self.x_right, self.x_right,
                 self.x_left],
                [self.y_bottom, self.y_top,
                 self.y_top, self.y_bottom,
                 self.y_bottom], 'r')

    # draw arena in whichever matplotlib plot was last used, or
    # a new window if there aren't any open
    def draw2(self):
        plt.plot([self.x_left, self.x_left,
                  self.x_right, self.x_right,
                  self.x_left],
                 [self.y_bottom, self.y_top,
                  self.y_top, self.y_bottom,
                  self.y_bottom], 'r')

    # draw arena in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        self.line_drawer(self.x_left, self.x_left, self.y_bottom, self.y_top,
                         screen, scale, shiftx, shifty)
        self.line_drawer(self.x_left, self.x_right, self.y_top, self.y_top,
                         screen, scale, shiftx, shifty)

        self.line_drawer(self.x_right, self.x_right, self.y_top, self.y_bottom,
                      screen, scale, shiftx, shifty)
        self.line_drawer(self.x_right, self.x_left, self.y_bottom, self.y_bottom,
                         screen, scale, shiftx, shifty)

    # a function for drawing a line in the pygame window
    def line_drawer(self, x1, x2, y1, y2, screen, scale, shiftx, shifty):
        pygame.draw.line(screen, color='green',
                         start_pos=(scale * x1 + shiftx, scale * y1 + shifty),
                         end_pos=(scale * x2 + shiftx, scale * y2 + shifty),
                         width=2)
