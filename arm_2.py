
import argparse
from math import pi
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller
import numpy as np
    

def find_smallest_distances(pairs, param1, param2, k):
    distances = np.array([abs(float(param1)  - (theta1)) + abs(float(param2) - (theta2)) for theta1, theta2 in pairs])
    sorted_indices = np.argsort(distances)
    return pairs[sorted_indices[:k]]   

# Run the following command for output: python arm_2.py --target 0 0 -k 3 --configs "arm_configs.npy"
if __name__=='__main__':
    # This code just gets us the name of the map from the command line
    parser = argparse.ArgumentParser(description="arm_1.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--configs', required=True, help='Path to the config file (e.g., "arm_configs.npy")')
    parser.add_argument('-k', required=True, )
    parser.add_argument('--target',  nargs=2, required=True, help='target orientation')
    args = parser.parse_args()

    ax = create_plot()
    planar_arm = Arm_Controller(0,0,ax, polygons=[])
    planar_arm.set_obs_plot()
    param1, param2 = args.target
    planar_arm.theta1, planar_arm.theta2 = float(param1), float(param2)
    planar_arm.re_orient()
    planar_arm.add_arm('b')
    configs = np.load(args.configs)
    param1, param2 = args.target
    smallest_distances = find_smallest_distances(configs, float(param1), float(param2), int(args.k))
    count = 0
    for theta1, theta2 in smallest_distances:
        planar_arm.theta1, planar_arm.theta2 = theta1, theta2
        planar_arm.re_orient()
        if count == 0:
            planar_arm.add_arm('r')
        elif count == 1:
            planar_arm.add_arm('g')
        elif count == 2:
            planar_arm.add_arm('b')
        else:
            planar_arm.add_arm('y')
        count+= 1
    planar_arm.theta1, planar_arm.theta2 = float(param1), float(param2)
    planar_arm.re_orient()
    planar_arm.add_arm('k')
    show_scene(ax)

    





