
import argparse
from math import pi, sqrt
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller
import numpy as np
from queue import PriorityQueue
    
# KD-Tree Node class
class Node:
    def __init__(self, point, split_dim):
        self.point = point
        self.split_dim = split_dim
        self.left = None
        self.right = None

# KD-Tree construction
def kd_tree(points, depth=0):
    if len(points) == 0:
        return None

    k = len(points[0])
    split_dim = depth % k

    # Note that it is not required to select the median point, In the case where median points are not selected, there is no guarantee that the tree will be balanced.
    points.sort(key=lambda x: x[split_dim])
    median = len(points) // 2

    node = Node(points[median], split_dim)
    node.left = kd_tree(points[:median], depth + 1)
    node.right = kd_tree(points[median + 1:], depth + 1)
    return node

def k_nearest_neighbors(root, query_point, k):
    pq = PriorityQueue()

    def search(node):
        nonlocal pq  # Make pq accessible within the nested function
        if node is None:
            return

        # Calculate distance
        dist = np.linalg.norm(np.array(query_point) - np.array(node.point))

        # Add to priority queue
        # used -dist for the distance metric in the priority queue. 
        # Therefore, the queue will now pop the element with the largest negative distance, 
        # which is the farthest point among the k closest points.
        if len(pq.queue) < k:
            pq.put((-dist, node.point))
        else:
            if -dist > pq.queue[0][0]:
                pq.get()
                pq.put((-dist, node.point))

        # Recursive search on child nodes
        split_dim = node.split_dim
        diff = query_point[split_dim] - node.point[split_dim]

        # If diff <= 0, it means that the query point lies on the 'left' side of the current node along that dimension. 
        # Thus, the 'left' subtree (i.e., node.left) becomes the primary search space, 
        # and the 'right' subtree (i.e., node.right) becomes the secondary one. The reverse is true if diff > 0.
        if diff <= 0:
            first, second = node.left, node.right
        else:
            first, second = node.right, node.left

        search(first)

        # abs(diff) < -pq.queue[0][0]: This is a pruning step. Recall that -pq.queue[0][0] is the smallest (negative) 
        # distance in the priority queue, i.e., the distance to the farthest point among the k closest points found so far. 
        # If abs(diff) is smaller than this distance, it means that there may exist points in the second subtree 
        # that are closer to the query point than the farthest point in the current set of k closest points. 
        # Therefore, it is necessary to explore the second subtree.
        if len(pq.queue) < k or abs(diff) < -pq.queue[0][0]:
            search(second)

    search(root)
    neighbors = [point for dist, point in sorted(pq.queue, key=lambda x: x[0], reverse=True)]

    return neighbors
# We plot all pairs of thetas as arms in the C-Space, compute the distances of each from our arm
# Sort and then return k closest pairs. This is done in O(nlogn) time
def find_smallest_distances(pairs, ax, arm, k):
    arms = [Arm_Controller(theta1, theta2, ax, polygons=[]) for theta1, theta2 in pairs]
    distances = np.array([find_distance(x, arm) for x in arms])
    sorted_indices = np.argsort(distances)
    return pairs[sorted_indices[:k]] 


def find_smallest_distances_kd(pairs, ax, arm, k):
    tree = kd_tree(pairs.tolist())
    nearest_points = k_nearest_neighbors(tree, (arm.theta1, arm.theta2), k)
    return nearest_points

# Distance between two arms is defined as sum of distance between their two joints
def find_distance(arm1, arm2):
    d1 = sqrt((arm1.joint2[0] - arm2.joint2[0])**2 + (arm1.joint2[1] - arm2.joint2[1])**2)
    d2 = sqrt((arm1.joint3[0] - arm2.joint3[0])**2 + (arm1.joint3[1] - arm2.joint3[1])**2)
    return d1 + d2

# Run the following command for output: python arm_2.py --target 0 0 -k 3 --configs "arm_configs.npy"
# Basically we set our arm to be at the target configuration, and then we search other configurations in "arm_configs.npy"
# To find the k nearest neighbors with a linear search
if __name__=='__main__':
    # This code just gets us the name of the map from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
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
    planar_arm.add_arm('b') # Original Target Arm
    configs = np.load(args.configs)
    param1, param2 = args.target
    smallest_distances = find_smallest_distances_kd(configs, ax, planar_arm, int(args.k))
    print(smallest_distances)
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

    





