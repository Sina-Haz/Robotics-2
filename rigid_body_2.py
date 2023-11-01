import matplotlib.patches as patches
import argparse
from math import pi, sqrt
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller
import numpy as np
from rigid_body_1 import make_rigid_body
    

def find_smallest_distances(configs, goal, k):
    distances = [find_distance(config, goal) for config in configs]
    sorted_indices = np.argsort(distances)
    return configs[sorted_indices[:k]] 

def find_distance(car1, car2):
    linear_distance = sqrt((car1[0] - car2[0])**2 + (car1[1] - car2[1])**2)
    angular_distance = abs(car1[2] % pi - car2[2] % pi)
    alpha = 0.7
    return alpha * linear_distance + (1-alpha) * angular_distance
# Run the following command for output: python arm_2.py --target 0 0 -k 3 --configs "arm_configs.npy"
if __name__=='__main__':
    # This code just gets us the name of the map from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--configs', required=True, help='Path to the config file (e.g., "arm_configs.npy")')
    parser.add_argument('-k', type=int, required=True, )
    parser.add_argument('--target', type=float, nargs=3, required=True, help='target orientation')
    args = parser.parse_args()
    ax = create_plot()
    x,y, theta = args.target
    configs = np.load(args.configs)
    car = make_rigid_body((x,y))
    smallest_distances = find_smallest_distances(configs, args.target, args.k)
    count = 0
    show_scene(ax)

    





