from arm_5 import Roadmap_Node, sample, get_k_neighbors, prm_animation_fn, collides, find_distance, interpolate, A_star
from planar_arm import Arm_Controller
import matplotlib.pyplot as plt
from typing import Callable
import random
import numpy as np
from create_scene import create_plot, load_polygons
from math import log2
import argparse


pi = np.pi

def PRM_star(iters: int, neighbors: Callable, sampler: Callable, robot: object, collides: Callable, dist_fn: Callable,
    discretize: Callable, startConfig: tuple, endConfig: tuple, config_lims:tuple, res: float, animation_fn: Callable):
    
    # We start off by creating a plot of the configuration space
    ax = create_plot()
    ax.set_xlim(config_lims[0], config_lims[1])
    ax.set_ylim(config_lims[0], config_lims[1])

    Roadmap = {} # A hashmap where key = config, value = Roadmap_Node obj
    Roadmap[startConfig] = Roadmap_Node(startConfig, edges=[], roads={})
    for i in range(iters):
        # Want to occasionally sample the end configuration to see if we can reach it, we assume it doesnt collide with anything
        if random.random() < 0.05:
            config = endConfig
        else:
            # Get a valid, non-colliding sample
            while True:
                config = sampler() # sample a point
                if not collides(robot, config): break
        
        # Append this configuration to Graph if doesn't already exist
        if Roadmap.get(config) == None:
            Roadmap[config] = Roadmap_Node(config, edges = [], roads = {}) 

        # To get closest neighbors we need a distance function that can compute distance between 2 configs
        # We feed it current config, all other vertices, k number of neighbors we want, and a distance function
        neighborhood = neighbors(config, [x for x in Roadmap.keys() if x != config],log2(len(Roadmap)), dist_fn) # Should return a list of nearby configurations
        
        for q in neighborhood:
            q = tuple(q)
            path = discretize(config, q, res) # Get a list of discretized configurations from sample to q
            if all(not collides(robot, pt) for pt in path):
                # Add edges between points, and also store the discretized path to get there
                Roadmap[config].edges.append(q)
                Roadmap[config].roads[q] = path
                Roadmap[q].edges.append(config)
                Roadmap[q].roads[config] = path[::-1]
        # If any valid edges exist we can add this sample to the configuration space and its edges
        if Roadmap[config].edges:
            animation_fn(config, Roadmap[config].edges, iters, ax)
    plt.show()
    return Roadmap

# python3 arm_5.py --start 0 0 --goal -2.9 0 --map "arm_polygons.npy"
if __name__ == '__main__':
    # This code gets us the inputs from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--start', type=float, nargs=2, required=True, help='start orientation')
    parser.add_argument('--map', required=True, help='Path to map file (e.g., "arm_polygons.npy")')
    parser.add_argument('--goal', type=float, nargs=2, required=True, help='target orientation')
    args = parser.parse_args()
    poly_map = load_polygons(args.map)


    planar_arm = Arm_Controller(args.start[0],args.start[1], ax = create_plot(), polygons = poly_map)
    graph = PRM_star(100, get_k_neighbors, sample, planar_arm, collides, 
                find_distance, interpolate, tuple(args.start), tuple(args.goal), [-pi, pi], np.radians(8), prm_animation_fn)
    
    var, path = A_star(tuple(args.start),tuple(args.goal), graph, find_distance)
    if var:
        all_points = []
        for i in range(len(path)-1):
            curr = path[i]
            next = path[i+1]
            road_to_next = graph[curr].roads[next]
            all_points += road_to_next
        planar_arm = Arm_Controller(args.start[0], args.start[1], ax = create_plot(), polygons=poly_map)
        planar_arm.set_obs_plot()
        planar_arm.draw_arm()
        for pt in all_points:
            planar_arm.set_joint_angles(pt)
            planar_arm.re_orient()
            planar_arm.ax.cla()
            planar_arm.draw_arm()
            planar_arm.ax.figure.canvas.draw()
            plt.pause(1e-3)
        # plt.close()
    print(f'goal is: {args.goal}, final position is: {(planar_arm.theta1, planar_arm.theta2)}')
    print('finished')