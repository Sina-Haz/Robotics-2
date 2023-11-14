import argparse
from math import pi, radians, floor, sqrt, ceil
import random
from rigid_body import CarController, check_car, check_boundary
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
        if prob < 0.8:
            return np.array([x,y, theta])
        else:
            return np.array(goal)
    
    #Travels the direction of the sampled node with the given step size
    def goToPoint(self, start, end, b, time, obstacles):
        offset = self.rho*self.unitVector(start, end)
        point = np.array([start.x + offset[0], start.y + offset[1], start.theta + offset[2]])
        blossom = []
        fig = plt.figure("Car")
        for i in range(b):
            v = random.uniform(-.5, .5)
            phi = random.uniform(-pi/4, pi/4)
            dynamic_car = Car(ax = fig.gca(), startConfig=(start.x, start.y, start.theta), dt = time)
            dynamic_car.set_velocity(v, phi)
            #print("Original car: ",dynamic_car.x, dynamic_car.y,dynamic_car.theta, dynamic_car.v, dynamic_car.phi)
            if not(self.isInObstacle(dynamic_car, obstacles)):
                attr = (v,phi, dynamic_car.x, dynamic_car.y, dynamic_car.theta)
                if self.goalFound(attr):
                    return np.array(attr)
                blossom.append(attr)
            #print(dynamic_car.x, dynamic_car.y, dynamic_car.theta, dynamic_car.v, dynamic_car.phi)
        closest = sorted(blossom, key = lambda b: self.distance((b[2], b[3], b[4]), point))[0]
        return np.array(closest)
    


    def getEdge(self, parent, child):
        for edge in parent.edges:
            if edge.get_child() == child:
                return edge

    #checks whether the node on the graph corresponds to a collision in the arm.
    def isInObstacle(self, dynamic_car, obstacles):
        dynamic_car.compute_next_position()
        if not check_car(dynamic_car.body, obstacles) and not check_boundary(dynamic_car.body):
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
        alpha = 0.7
        dist = alpha * self.linear_distance(point1,point2) + (1-alpha) * self.angular_distance(point1,point2)
        return dist
    

    def linear_distance(self,point1, point2):
        linear_distance = sqrt((point1[0] - point2[0])**2 + (point1[1]- point2[1])**2)
        return linear_distance
    
    def angular_distance(self, point1, point2):
        angular_distance = abs(angle_mod(point1[2])- angle_mod(point2[2]))
        return angular_distance
    
    def goalFound(self, point):
        a = self.angular_distance((self.goal.x, self.goal.y, self.goal.theta), (point[2], point[3], point[4]))
        if self.linear_distance((self.goal.x, self.goal.y, self.goal.theta), (point[2], point[3], point[4])) <= .1 and a <0.5:
            return True
    

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retracePath(self, goal):
        if goal.x == self.randomTree.x and goal.y == self.randomTree.y and goal.theta == self.randomTree.theta:
            return
        self.numWaypoints += 1
        edge = self.getEdge(goal.parent, goal)
        currentPoint = np.array([goal.x, goal.y, goal.theta, edge.v, edge.phi])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.rho
        self.currentNode = goal
        self.retracePath(goal.parent)
        
    

def rtt_tree(start, goal, obstacles):
    plt.close('all')
    fig = plt.figure("RTT Algorithm")
    plt.plot(start[0], start[1], 'ro', alpha=0.1)
    ax = fig.gca()
    ax.set_xlim(0,2)
    t = .2
    ax.set_ylim(0,2)
    rrt = RTT(start, goal, 1000, t)
    i = 0
    while i < 1000:
        print("Iteration: ", i)
        rrt.resetNearestValues()
        point = rrt.samplePoint(goal)
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.goToPoint(rrt.nearestNode, point, 200, t, obstacles)
        i += 1
        rrt.addChild(new[0], new[1], new[2], new[3], new[4])
        #plt.pause(.01)
        #plt.plot([rrt.nearestNode.x, new[2]],[rrt.nearestNode.y, new[3]], 'go', linestyle="--")
        if rrt.goalFound(new):
            rrt.addChild(0,0, goal[0], goal[1], goal[2])
            print("Goal Found")
            break
    #plt.pause(1)
    rrt.retracePath(rrt.goal)
    edge = rrt.getEdge(rrt.randomTree, rrt.currentNode)
    rrt.Waypoints.insert(0, (start[0], start[1], start[2], edge.v, edge.phi))
    for i in range(len(rrt.Waypoints) - 1):
        break
        #plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle="--")
    #plt.pause(1)
    plt.close()
    fig,ax = plt.subplots(dpi=100)
    car_graph(ax, rrt.Waypoints,start, obstacles, t)
    


def car_graph(ax, waypoints,start, obstacles, t):
    car =  Car(ax = ax, startConfig=(start[0], start[1], start[2]), dt = t)
    car.set_obs(obstacles)
    car.set_obs_plot()
    car.start_animation(len(waypoints), 500, waypoints)
    
    

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

    


