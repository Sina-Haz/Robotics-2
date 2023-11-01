import argparse
from math import pi, radians, floor
import random
from arm import NLinkArm
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from arm_1 import get_sample
from arm_3 import interpolate
from planar_arm import Arm_Controller
import numpy as np
import matplotlib.pyplot as plt

class treeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.children = []
        self.parent = None

class RTT():
    def __init__(self, start, goal, iterations, stepSize):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(iterations, 1000)
        self.rho = stepSize
        self.path_distance = 0
        self.nearestDist = 10000
        self.numWaypoints = 0
        self.Waypoints = []
    
    #Add point to nearest node. If goal node, return
    def addChild(self, x, y):
        if (x == self.goal.x):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(x, y)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
    
    def samplePoint(self):
        x = random.random()*2*pi
        y = random.random()*2*pi
        point = np.array([x,y])
        return point
    
    def goToPoint(self, start, end):
        offset = self.rho*self.unitVector(start, end)
        print(start.x)
        point = np.array([start.x + offset[0], start.y + offset[1]])
        return point
    
    def isInObstacle(self, start, end):
        u_hat = self.unitVector(start, end)
        test = np.array([0.0,0.0])
        for i in range(floor(self.rho)):
            test[0] = start.x + i* u_hat[0]
            test[1] = start.y + i * u_hat[1]
            #check if point in obstacle
            return True
        return False
    
    def unitVector(self, start, end):
        v = np.array([end[0] - start.x, end[1] - start.y])
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
        dist = np.sqrt((node1.x - point[0])**2 + (node1.y - point[1]))
        return dist
    
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
    

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retracePath(self, goal):
        if goal.x == self.randomTree.x:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.x, goal.y])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.rho
        self.retracePath(goal.parent)
        
    

def rtt_tree(start, goal):
    fig = plt.figure("RTT Algorithm")
    plt.plot(start[0], start[1], 'ro')
    ax = fig.gca()
    ax.set_xlim(0,2*pi)
    ax.set_ylim(0,2*pi)
    rrt = RTT(start, goal, 100, 0.1)
    for i in range(100):
        rrt.resetNearestValues()
        point = rrt.samplePoint()
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.goToPoint(rrt.nearestNode, point)
        bool = rrt.isInObstacle(rrt.nearestNode, new)
        if bool == False:
            rrt.addChild(new[0], new[1])
            plt.pause(0.1)
            plt.plot([rrt.nearestNode.x, new[0]],[rrt.nearestNode.y, new[1]], 'go', linestyle="--")
            if rrt.goalFound(new):
                rrt.addChild(goal[0], goal[1])
                print("Goal Found")
                break



def generate_sample(arm, start, goal):
    theta1 = random.random()*2*pi
    theta2 = random.random()*2*pi
    discretized_pts = interpolate(start, (theta1, theta2), radians(5))
    for pt in discretized_pts:
        arm.set_joint_angles(pt)
        arm.re_orient()
        arm.ax.cla()
        arm.draw_arm()
        arm.ax.figure.canvas.draw()
    

if __name__=='__main__':
    # This code gets us the inputs from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--start', type=float, nargs=2, required=True, help='start orientation')
    parser.add_argument('--map', required=True, )
    parser.add_argument('--goal', type=float, nargs=2, required=True, help='target orientation')
    args = parser.parse_args()
    
    #get the parameters from the parser
    start1, start2 = args.start
    goal1, goal2 = args.goal
    poly_map = load_polygons(args.map)
    
    #Create the plot
    rtt_tree(np.array([start1, start2]), np.array([goal1, goal2]))
    '''
    tree = RTT((start1,start2))
    counter = 0
    planar_arm = Arm_Controller(args.start[0], args.start[1])
    planar_arm.set_arm_obs(poly_map)
    planar_arm.set_obs_plot()
    generate_sample(planar_arm, args.start, args.goal)
    '''


