import sys
# path folder which contains situsim_v1_2
sys.path.insert(1, '..')
from situsim_v1_2 import *

# a class which allows use to draw LightSources in different colours
# - it adds and changes nothing else
class ColouredLightSource(LightSource):

    # construct light source
    def __init__(self, x, y, colour, brightness=1, gradient=0.01, model='inv_sq', is_on=True, theta=None):
        super().__init__(x, y, theta, brightness, gradient, model, is_on)
        self.colour = colour

    # draw light source in the specified matplotlib axes
    def draw(self, ax):
        if self.is_on:
            ax.add_artist(mpatches.Circle((self.x, self.y), 0.7, color=self.colour))
            ax.add_artist(mpatches.Circle((self.x, self.y), 0.2, color='orange'))
            ax.plot(self.x, self.y, 'r.')

    # draw light source in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        if self.is_on:
            pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color=self.colour, radius=scale*0.7)
            pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color='orange', radius=scale*0.2)
