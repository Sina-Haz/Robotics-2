import matplotlib.patches as patches
import argparse
from math import pi, sqrt
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller, angle_mod
import numpy as np
from rigid_body_1 import make_rigid_body
import matplotlib.pyplot as plt
    

def find_smallest_distances(configs, goal, k):
    configs = np.array(configs)
    distances = np.array([find_distance(config, goal) for config in configs])
    sorted_indices = np.argsort(distances)
    return configs[sorted_indices[:k]] 

def find_distance(car1, car2):
    linear_distance = sqrt((car1[0] - car2[0])**2 + (car1[1] - car2[1])**2)
    angular_distance = abs(angle_mod(car1[2])- angle_mod(car1[2]))
    alpha = 0.7
    return alpha * linear_distance + (1-alpha) * angular_distance


# Run the following command for output: python3 rigid_body_2.py --target 1 1 0 --k 3 --configs "rigid_configs.npy"
if __name__=='__main__':
    # This code just gets us the name of the map from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--configs', required=True, help='Path to the config file (e.g., "arm_configs.npy")')
    parser.add_argument('--k', type=int, required=True, )
    parser.add_argument('--target', type=float, nargs=3, required=True, help='target orientation')
    args = parser.parse_args()
    plt.close('all')
    ax = create_plot()
    x,y, theta = args.target
    configs = np.load(args.configs)
    car = make_rigid_body((x,y))
    ax.add_patch(car)
    smallest_distances = find_smallest_distances(configs, args.target, args.k)
    count = 0
    for rectangle in smallest_distances:
        body = make_rigid_body((rectangle[0], rectangle[1]), rectangle[2], 0.5)
        if count == 0: body.set_facecolor('red')
        elif count == 1: body.set_facecolor('green')
        elif count == 2: body.set_facecolor('blue')
        else: body.set_facecolor('yellow')
        ax.add_patch(body)
        count += 1
    show_scene(ax)

    





