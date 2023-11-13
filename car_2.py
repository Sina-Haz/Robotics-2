import argparse
from math import pi, radians, floor, sqrt
import random
from rigid_body import CarController, check_car
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from rigid_body_1 import make_rigid_body
from rigid_body_3 import interpolate, reposition_car
from planar_arm import angle_mod
from car_1 import Car
import numpy as np
import matplotlib.pyplot as plt


class Edge:
    def __init__(self, begin, end, v, phi):
        self.begin = begin
        self.end = end
        self.v = v
        self.phi = phi
    
    def get_child(self):
        return self.end
#represents a node that is plotted on the tree graph.
class treeNode:
    def __init__(self, x, y, theta):
        self.x = x #x coord 
        self.y = y  #ycoord
        self.theta = theta  #angle of rigid body
        self.edges = []    #all neighboring nodes
        self.parent = None     

#Represents the entire RRT tree
class RTT():
    def __init__(self, start, goal, iterations, stepSize):
        self.randomTree = treeNode(start[0], start[1], start[2])  
        self.goal = treeNode(goal[0], goal[1], goal[2])
        self.nearestNode = None
        self.currentNode = None
        self.iterations = min(iterations, 1000)
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
    
    #Add point to nearest node. If goal node, return
    def addChild(self, v, phi, x, y, theta):
        if x == self.goal.x and y == self.goal.y and theta == self.goal.theta:
            edge = Edge(self.nearestNode, self.goal, v, phi)
            self.nearestNode.edges.append(edge)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(x, y, theta)
            edge = Edge(self.nearestNode, tempNode, v, phi)
            self.nearestNode.edges.append(edge)
            tempNode.parent = self.nearestNode
            self.currentNode = tempNode
    
    #Samples a random point between -pi and pi. Occasionally samples the goal node with a 5% probability
    def samplePoint(self, goal):
        x = random.uniform(0.1,1.9)
        y = random.uniform(0.1,1.9)
        theta = random.uniform(-pi, pi)
        prob = random.uniform(0, 1)
        if prob < 0.9:
            return np.array([x,y, theta])
        else:
            return np.array(goal)
    
    #Travels the direction of the sampled node with the given step size
    def goToPoint(self, start, end, b, time, obstacles):
        offset = self.rho*self.unitVector(start, end)
        point = np.array([start.x + offset[0], start.y + offset[1], start.theta + offset[2]])
        blossom = []
        for i in range(b):
            v = random.uniform(-.5, .5)
            phi = random.uniform(-pi/4, pi/4)
            dynamic_car = Car(None, startConfig=(start.x, start.y, start.theta), dt = .1)
            dynamic_car.set_velocity(v, phi)
            if not(self.isInObstacle(dynamic_car, obstacles, time, 0.1)):
                blossom.append((v,phi, dynamic_car.x, dynamic_car.y, dynamic_car.theta))
        closest = sorted(blossom, key = lambda b: self.distance((b[2], b[3], b[4]), point))[0]
        #print(closest)
        return np.array(closest)
    


    #checks whether the node on the graph corresponds to a collision in the arm.
    def isInObstacle(self, dynamic_car, obstacles, t, dt):
        for j in range(int(t/dt)):
            new = dynamic_car.compute_next_position()
            if not check_car(make_rigid_body((new[0], new[1]), new[2]), obstacles):
                return True
        return False
    
    def unitVector(self, start, end):
        v = np.array([end[0] - start.x, end[1] - start.y, end[2] - start.theta])
        u_hat = v/np.linalg.norm(v)
        return u_hat
    
    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance((root.x, root.y, root.theta),point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for edge in root.edges:
            child = edge.get_child()
            self.findNearest(child,point)

    def distance(self, point1,point2):
        #dist = np.sqrt((node1.x - point[0])**2 + (node1.y - point[1])**2)
        linear_distance = sqrt((point1[0] - point2[0])**2 + (point1[1]- point2[1])**2)
        angular_distance = abs(angle_mod(point1[2])- angle_mod(point2[2]))
        alpha = 0.7
        dist = alpha * linear_distance + (1-alpha) * angular_distance
        return dist
    
    def goalFound(self, point, obstacles):
        if self.distance((self.goal.x, self.goal.y, self.goal.theta), point) <= 1:
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
    rrt = RTT(start, goal, 1000, .3)
    i = 0
    while i < 1000:
        rrt.resetNearestValues()
        point = rrt.samplePoint(goal)
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.goToPoint(rrt.nearestNode, point, 20, .8, obstacles)
        i += 1
        rrt.addChild(new[0], new[1], new[2], new[3], new[4])
        if rrt.goalFound(new, obstacles):
            rrt.addChild(0,0, goal[0], goal[1], goal[2])
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
        if(not check_car(rig_body.car, obstacles)):
            print("true")
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

    


