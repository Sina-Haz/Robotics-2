from arm_5 import PRM, Roadmap_Node
from math import pi, degrees, radians
from create_scene import create_plot, load_polygons
import random, heapq
from rigid_body_2 import find_smallest_distances, find_distance
from rigid_body import CarController,check_car, check_boundary
from rigid_body_3 import reposition_car, interpolate
from rigid_body_1 import make_rigid_body
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# samples random configuration (x,y,theta)
def sample():
    rand_x = random.random()*2
    rand_y = random.random()*2
    rand_theta = random.uniform(-pi, pi)
    return rand_x,rand_y, rand_theta

# Just reordering so we can reuse code
def get_k_neighbors(currConfig, otherConfigs, k, dist_fn = None):
    return find_smallest_distances(otherConfigs, currConfig, k)

# Assumes we already added the obstacles as an instance of the CarController obj
def collides(rigid_body: CarController, config):
    reposition_car(config, rigid_body)
    return not (check_car(rigid_body.car, rigid_body.obstacles) and check_boundary(rigid_body.car))

# Gimmick animation function for PRM, will only use x, y and ignore theta. Just want to make sure PRM working properly
def prm_animation_fn(config, edges, iters, ax):
    pass
    # x1,y1,_ = config
    # plt.scatter(x1, y1,c='g')
    # for edge in edges:
    #     x2,y2,_ = edge
    #     plt.plot([x1,x2], [y1,y2], c='g')
    # if iters % 50 == 0:
    #     ax.figure.canvas.draw()
    #     plt.pause(1e-6)
    

# Almost the same A star as we have in arm 5 with a small tweak for distance function
def A_star(startConfig, goalConfig, Graph, dist_fn):
    def get_dist(config1, config2):
        return dist_fn(config1,config2)
    
    def h(config):
        return get_dist(config, goalConfig)
    
    def get_path(config):
        path = [config]
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
            return True, get_path(goalConfig)

        for child in Graph[curr[1]].edges:
            tmpDist = distances[curr[1]] + get_dist(curr[1], child)
            if child not in distances or tmpDist < distances[child]:
                distances[child] = tmpDist
                parents[child] = curr[1]
                fringe.append((tmpDist+h(child), child))
    return False, []

def dfs(graph, src, goal, visited=None, path=None):
    if visited is None:
        visited = set()
    if path is None:
        path = []
    path = path+[src]
    visited.add(src)
    if src == goal:
        return path
    for neighbor in graph[src].edges:
        if neighbor not in visited:
            new_path = dfs(graph, neighbor, goal, visited, path)
            if new_path: return new_path
    return None


# Usage: python3 rigid_body_5.py --start 0.5 0.25 0 --goal 1.75 1.5 0.5 --map "rigid_polygons.npy"
if __name__ == '__main__':
    # This code gets us the inputs from the command line
    parser = argparse.ArgumentParser(description="rigid_body_5 finds collision free path between two configs using PRM")
    parser.add_argument('--start', type=float, nargs=3, required=True, help='start orientation')
    parser.add_argument('--map', required=True, help='Path to map file (e.g., "arm_polygons.npy")')
    parser.add_argument('--goal', type=float, nargs=3, required=True, help='target orientation')
    args = parser.parse_args()
    poly_map = load_polygons(args.map)

    rig_body = CarController(ax = create_plot(), car = make_rigid_body((args.start[:2])), obstacles=[])
    rig_body.car.set_angle(degrees(args.start[2]))
    rig_body.set_obstacles(poly_map)

    graph = PRM(300, get_k_neighbors, 3, sample, rig_body, collides, find_distance, interpolate,
                tuple(args.start), tuple(args.goal), (0, 2), 0.05, prm_animation_fn)
    print('PRM finished')
    plt.close()
    
    # Reset the rigid body once we have the PRM graph
    rig_body2 = CarController(ax = create_plot(), car = make_rigid_body((args.start[:2])), obstacles=[])
    rig_body2.car.set_angle(degrees(args.start[2]))
    rig_body2.set_obstacles(poly_map)

    def update(frame, discretized_pts, rig_body):
        pt = discretized_pts[frame]
        reposition_car(pt, rig_body)
        rig_body.ax.cla()
        rig_body.ax.set_ylim([0, 2])
        rig_body.ax.set_xlim([0, 2])
        rig_body.ax.add_patch(rig_body.car)
        rig_body.set_obstacles(rig_body.obstacles)

    
    var, path = A_star(tuple(args.start),tuple(args.goal), graph, find_distance)
    if var:
        all_points = []
        for i in range(len(path)-1):
            curr = path[i]
            next = path[i+1]
            road_to_next = graph[curr].roads[next]
            all_points += road_to_next
        all_points.append(path[-1])
        # for pt in all_points:
        #     reposition_car(pt, rig_body2)
        #     rig_body2.ax.cla()
        #     rig_body2.ax.set_ylim([0,2])
        #     rig_body2.ax.set_xlim([0,2])
        #     rig_body2.ax.add_patch(rig_body2.car)
        #     rig_body2.set_obstacles(rig_body2.obstacles)
        #     plt.draw()
        #     plt.pause(1e-4)
        #     rig_body2.ax.figure.canvas.draw()
        ani = FuncAnimation(rig_body2.fig, update, frames=len(all_points),
                fargs=(all_points, rig_body2), interval=100, blit=False, repeat=False)
        ani.save('videos/rigid_body2.5-4.mp4', 'ffmpeg', fps=30)
        plt.show()
    else:
        print('no path exists')
    print('finished')
    
        
    




