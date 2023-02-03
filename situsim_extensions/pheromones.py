import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
from situsim_v1_2 import *

# subclass LightSource to create a LightSource which has a quantity. The quantity decays from 1 towards 0, is controlled
# by a PheromoneManager, and in this class it affects the brightness of the light, as perceived by LightSensors, and
# the opacity of the light when it is drawn in pygame and matplotlib
class PheromoneSource(LightSource):

    # construct pheromone
    def __init__(self, x, y, brightness=1, gradient=0.01, model='inv_sq', is_on=True):
        super().__init__(x=x, y=y, brightness=brightness, gradient=gradient, model=model, is_on=is_on)
        self.quantity = 1

    # as the quantity of pheromone decays, the light dims
    def get_brightness_at(self, x, y):
        return self.quantity * super().get_brightness_at(x, y)

    # draw pheromone in the specified matplotlib axes
    def draw(self, ax):
        ax.add_artist(mpatches.Circle((self.x, self.y), 0.7, color='yellow', alpha=self.quantity))
        ax.add_artist(mpatches.Circle((self.x, self.y), 0.2, color='orange', alpha=self.quantity))
        ax.plot(self.x, self.y, 'r.')

    # draw pheromone in a pygame display
    def pygame_draw(self, screen, scale, shiftx, shifty):
        width = 0
        # pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color=pygame.Color(255, int(255*(1-self.quantity)), int(255*(1-self.quantity))), width=width, radius=scale*0.7)
        pygame.draw.circle(screen, center=(scale*self.x+shiftx, scale*self.y+shifty), color=pygame.Color(int(255*self.quantity), 0, 0), width=width, radius=scale*0.7)


# a class to handle a collection of pheromones, which can be added to and removed when they decay to 0
class PheromoneManager:

    # construct pheromone manager
    def __init__(self, pheromones=[], decay_rate=0.01):
        self.pheromones = pheromones
        self.decay_rate = decay_rate  # rate of pheromone decay per unit of simulation time

    # add pheromone at given coordinate
    def add_pheromone_at(self, x, y):
        self.pheromones.append(PheromoneSource(x, y))

    # add an already existing PheromoneSource (not currently used)
    def add_pheromone(self, p):
        self.pheromones.append(p)

    # step pheromone manager
    def step(self, dt):
        for p in self.pheromones:
            p.quantity -= dt * self.decay_rate  # all pheromones decay by the same amount
        self.pheromones[:] = [p for p in self.pheromones if p.quantity > 0]  # remove all pheromones which are depleted

    # draw all pheromones in specified matplotlib axes
    def draw(self, ax):
        for p in self.pheromones:
            p.draw(ax)

    # draw all pheromones in pygame screen
    def pygame_draw(self, screen, scale, shiftx, shifty):
        for p in self.pheromones:
            p.pygame_draw(screen, scale, shiftx, shifty)
