import argparse
from math import pi, radians, floor, sqrt
import random
from rigid_body import CarController, check_car
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from rigid_body_1 import make_rigid_body
from rigid_body_3 import interpolate, reposition_car
from planar_arm import angle_mod
import numpy as np
import matplotlib.pyplot as plt

#represents a node that is plotted on the tree graph.
class treeNode:
    def __init__(self, x, y, theta):
        self.x = x #x coord 
        self.y = y  #ycoord
        self.theta = theta  #angle of rigid body
        self.children = []    #all neighboring nodes
        self.parent = None     

#Represents the entire RRT tree
class RTT():
    def __init__(self, start, goal, iterations, stepSize):
        self.randomTree = treeNode(start[0], start[1], start[2])  
        self.goal = treeNode(goal[0], goal[1], goal[2])
        self.nearestNode = None
        self.iterations = min(iterations, 1000)
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
    
    #Add point to nearest node. If goal node, return
    def addChild(self, x, y, theta):
        if x == self.goal.x and y == self.goal.y and theta == self.goal.theta:
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(x, y, theta)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
    
    #Samples a random point between -pi and pi. Occasionally samples the goal node with a 5% probability
    def samplePoint(self, goal):
        x = random.uniform(0,2)
        y = random.uniform(0,2)
        theta = random.uniform(-pi, pi)
        prob = random.uniform(0, 1)
        if prob < 0.9:
            return np.array([x,y, theta])
        else:
            return np.array(goal)
    
    #Travels the direction of the sampled node with the given step size
    def goToPoint(self, start, end):
        offset = self.rho*self.unitVector(start, end)
        point = np.array([start.x + offset[0], start.y + offset[1], start.theta + offset[2]])
        return point
    
    #checks whether the node on the graph corresponds to a collision in the arm.
    def isInObstacle(self, start, end, obstacles):
        u_hat = self.unitVector(start, end)
        points = interpolate((start.x, start.y, start.theta), (u_hat[0], u_hat[1], u_hat[2]))
        for point in points:
            if not check_car(make_rigid_body((point[0], point[1]), point[2]), obstacles):
                return True   
        return False
    
    def unitVector(self, start, end):
        v = np.array([end[0] - start.x, end[1] - start.y, end[2] - start.theta])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root,point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child,point)

    def distance(self, node1,point):
        #dist = np.sqrt((node1.x - point[0])**2 + (node1.y - point[1])**2)
        linear_distance = sqrt((node1.x - point[0])**2 + (node1.y- point[1])**2)
        angular_distance = abs(angle_mod(node1.theta)- angle_mod(point[2]))
        alpha = 0.7
        dist = alpha * linear_distance + (1-alpha) * angular_distance
        return dist
    
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
    

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retracePath(self, goal):
        if goal.x == self.randomTree.x and goal.y == self.randomTree.y and goal.theta == self.randomTree.theta:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.x, goal.y, goal.theta])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.rho
        self.retracePath(goal.parent)
        
    

def rtt_tree(start, goal, obstacles):
    plt.close('all')
    fig = plt.figure("RTT Algorithm")
    plt.plot(start[0], start[1], 'ro', alpha=0.1)
    ax = fig.gca()
    ax.set_xlim(0,2)
    ax.set_ylim(0,2)
    rrt = RTT(start, goal, 1000, .2)
    i = 0
    while i < 1000:
        rrt.resetNearestValues()
        point = rrt.samplePoint(goal)
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.goToPoint(rrt.nearestNode, point)
        bool = rrt.isInObstacle(rrt.nearestNode, new, obstacles)
        if bool == False:
            i += 1
            #plt.pause(0.001)
            #plt.plot([rrt.nearestNode.x, new[0]],[rrt.nearestNode.theta, new[2]], 'go', linestyle="--")
            rrt.addChild(new[0], new[1], new[2])
            if rrt.goalFound(new):
                rrt.addChild(goal[0], goal[1], goal[2])
                print("Goal Found")
                break
    plt.pause(1)
    rrt.retracePath(rrt.goal)
    rrt.Waypoints.insert(0, start)
    for i in range(len(rrt.Waypoints) - 1):
        plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle="--")
    plt.pause(1)
    plt.close()
    fig,ax = plt.subplots(dpi=100)
    rig_body = CarController(ax, car = make_rigid_body(start), obstacles=[])
    rigid_graph(start, rig_body, rrt.Waypoints, obstacles)


def rigid_graph(start,rig_body, waypoints, obstacles):
    begin = start
    for i in range(1, len(waypoints)):
        move_rigid(rig_body, begin, waypoints[i], obstacles)
        begin = waypoints[i]


def move_rigid(rig_body, start, goal, obstacles):
    discretized_pts = interpolate(start, goal, 0.01)
    for pt in discretized_pts:
        print(pt)
        reposition_car(pt, rig_body)
        rig_body.ax.cla()
        rig_body.set_obstacles(obstacles)
        rig_body.ax.set_ylim([0,2])
        rig_body.ax.set_xlim([0,2])
        rig_body.ax.add_patch(rig_body.car)
        plt.draw()
        plt.pause(1e-5)
        rig_body.ax.figure.canvas.draw()
    print('\n')
    

if __name__=='__main__':
    # This code gets us the inputs from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--start', type=float, nargs=3, required=True, help='start orientation')
    parser.add_argument('--map', required=True, )
    parser.add_argument('--goal', type=float, nargs=3, required=True, help='target orientation')
    args = parser.parse_args()
    
    #get the parameters from the parser
    start1, start2, theta1 = args.start[0], args.start[1], angle_mod(args.start[2])
    goal1, goal2, theta2 = args.goal[0], args.goal[1], angle_mod(args.goal[2])
    poly_map = load_polygons(args.map)
    

    rtt_tree(np.array([start1, start2, theta1]), np.array([goal1, goal2, theta2]), poly_map)

    


