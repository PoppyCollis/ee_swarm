from .sensors import *
from .motors import *

class Ant(Agent):

    def __init__(self, x, y, controller,):

        self.x = x
        self.y = y
        self.controller = controller


