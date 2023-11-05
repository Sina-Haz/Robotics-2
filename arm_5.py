from planar_arm import Arm_Controller
import argparse
from create_scene import create_plot, load_polygons
import numpy as np
import random
import matplotlib.pyplot as plt
from arm_2 import find_distance
from arm_3 import interpolate
import heapq
from typing import Callable

pi = np.pi

# Implement PRM algorithm to get from start to goal
# Add 1000 nodes to the graph
# Try to connect with k = 3 closest neighbors

# Once you have graph from PRM, need to run A* or Dijkstra to get path

# Include: 
    # An animation that shows the roadmap data structure generated by PRM growing over the
    # search process in the arm’s configuration space, i.e., each node of PRM corresponds to a
    # point in the configuration space and edges correspond to straight line segments. Make sure
    # that your animation properly treats the topology of the configuration space

    # An animation that shows the arm moving along the solution path inside the original workspace
    # from the start configuration to the goal configuration. This path should not be resulting in col-
    # lisions with the environment.

class Roadmap_Node:
    def __init__(self,config: tuple, edges: list, roads: dict):
        self.config = config # This is an n-tuple where n = # dofs for a configuration
        self.edges = edges # This is a list of configurations that this node has edges to
        self.roads = roads # This is a hashmap where key = config we have an edge to, value = list of discretized points
        # that start at current config and end at edge config

        
"""# We map [-pi, pi] -> [0, 2pi] -> [0,1] -> [0, 100]
def config_to_indices(theta1, theta2):
    i = int(((theta1 + pi) / (2 * pi)) * 100)
    j = int(((theta2 + pi) / (2 * pi)) * 100)
    return i, j

""# Undoing previous operations to map from [0, 100] -> [0, 1] -> [0, 2pi] -> [-pi, pi]
def indices_to_config(i, j):
    theta1 = (i / 100) * 2 * pi - pi
    theta2 = (j / 100) * 2 * pi - pi
    return theta1, theta2"""

# Get's us theta in range [-pi, pi]
def sample():
    theta1 = (random.random()*2*pi) - pi
    theta2 = (random.random()*2*pi) - pi
    return theta1,theta2

# Getting k closest neighbors in T^2
def get_k_neighbors(config, other_configs, k, dist_fn):
    other_configs = np.array(other_configs)
    t1,t2 = config
    original = Arm_Controller(t1,t2)
    others = [Arm_Controller(theta1, theta2) for theta1,theta2 in other_configs]
    all_dist = np.array([dist_fn(x, original) for x in others])
    sorted_indices = np.argsort(all_dist)
    return other_configs[sorted_indices[:k]]

def collides(planar_arm: Arm_Controller, config):
    planar_arm.set_joint_angles(config)
    planar_arm.re_orient()
    return any(planar_arm.check_arm_collisions())


# Returns a graph in adjacency list representation
def PRM(iters: int, neighbors: Callable, k: int, sampler: Callable, robot: object, collides: Callable, dist_fn: Callable,
    discretize: Callable, startConfig: tuple, endConfig: tuple, config_lims:tuple, res: float):
    
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
            Roadmap[config] = Roadmap_Node(config, edges = [], roads = {}) # Is this an issue now that sometimes config = endConfig???

        # To get closest neighbors we need a distance function that can compute distance between 2 configs
        # We feed it current config, all other vertices, k number of neighbors we want, and a distance function
        neighborhood = neighbors(config, [x for x in Roadmap.keys() if x != config], k, dist_fn) # Should return a list of nearby configurations
        
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
            x1,y1 = config
            plt.scatter(x1, y1,c='g')
            for edge in Roadmap[config].edges:
                x2,y2 = edge
                plt.plot([x1,x2], [y1,y2], c='g')
            if iters % 50 == 0:
                ax.figure.canvas.draw()
                plt.pause(1e-6)
    plt.show()
    plt.close(ax.figure)
    return Roadmap

# Assumes a graph which is a hashmap of Roadmap_Node objects
def A_star(startConfig, goalConfig, Graph, dist_fn):
    def arm_dist(config1, config2):
        x1,y1 = config1
        x2,y2 = config2
        arm1 = Arm_Controller(x1,y1)
        arm2 = Arm_Controller(x2,y2)
        return dist_fn(arm1,arm2)
    
    def h(config):
        return arm_dist(config, goalConfig)
    
    def get_path(config):
        path = []
        while parents[config]:
            path.append(parents[config])
            config = parents[config]
        return path[::-1]
    
    distances = {startConfig:0}
    parents = {startConfig:None}
    fringe = [(h(startConfig), startConfig)] #PQ, stores tuples (priority = dist + heuristic, configuration)
    while fringe:
        curr = heapq.heappop(fringe)
        if curr[1] == goalConfig:
            return True, get_path(curr[1])

        for child in Graph[curr[1]].edges:
            tmpDist = distances[curr[1]] + arm_dist(curr[1], child)
            if child not in distances or tmpDist < distances[child]:
                distances[child] = tmpDist
                parents[child] = curr[1]
                fringe.append((tmpDist+h(child), child))
    return False, []


        


# Usage: python3 arm_5.py --start 0 0 --goal -2.9 0 --map "arm_polygons.npy"
if __name__ == '__main__':
    # This code gets us the inputs from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--start', type=float, nargs=2, required=True, help='start orientation')
    parser.add_argument('--map', required=True, help='Path to map file (e.g., "arm_polygons.npy")')
    parser.add_argument('--goal', type=float, nargs=2, required=True, help='target orientation')
    args = parser.parse_args()
    poly_map = load_polygons(args.map)


    planar_arm = Arm_Controller(0, 0, ax = create_plot(), polygons = poly_map)
    graph = PRM(100, get_k_neighbors, 3, sample, planar_arm, collides, 
                find_distance, interpolate, tuple(args.start), tuple(args.goal), [-pi, pi], np.radians(8))
    
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
            plt.pause(1e-4)
        # plt.close()
        print('finished')
        

    # planar_arm.set_obs_plot()

    # # Our configuration space will be a discretized grid of 100*100
    # # config_space = np.zeros((100, 100))
    # ax = create_plot()
    # ax.set_xlim(-pi, pi)
    # ax.set_ylim(-pi, pi)

    # configs = [] #Store sampled configurations here
    # for i in range(1,11):
    #     t1,t2 = sample()
    #     configs.append((t1,t2))
    #     plt.scatter(t1, t2, c='g', label='Sampled Configurations')
    #     plt.pause(1e-5)
    # x1,y1 = configs[0]
    # x2,y2 = configs[1]
    # plt.plot([x1,x2], [y1,y2], c='g')
    # ax.figure.canvas.draw()
    # plt.show()

