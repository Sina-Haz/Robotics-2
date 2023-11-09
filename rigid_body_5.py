from arm_5 import Roadmap_Node
from math import pi, degrees
import random
from rigid_body_2 import find_smallest_distances
from rigid_body import CarController,check_car
from rigid_body_3 import reposition_car

# samples random configuration (x,y,theta)
def sample():
    rand_x = random.random()*2
    rand_y = random.random()*2
    rand_theta = random.randrange(-pi, pi)
    return rand_x,rand_y, rand_theta

# Just reordering so we can reuse code
def get_k_neighbors(currConfig, otherConfigs, k):
    return find_smallest_distances(otherConfigs, currConfig, k)

# Assumes we already added the obstacles as an instance of the CarController obj
def collides(rigid_body: CarController, config):
    reposition_car(config, rigid_body)
    return check_car(rigid_body.car, rigid_body.obstacles)





# Usage: python3 rigid_body_5.py --start 0.5 0.25 0 --goal 1.75 1.5 0.5 --map "rigid_polygons.npy"
