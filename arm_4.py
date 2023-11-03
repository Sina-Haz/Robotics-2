import argparse
from math import pi, radians, floor
import random
from arm import NLinkArm
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from arm_1 import get_sample
from arm_3 import interpolate
from planar_arm import Arm_Controller, angle_mod
import numpy as np
import matplotlib.pyplot as plt

#represents a node that is plotted on the tree graph.
class treeNode:
    def __init__(self, x, y):
        self.x = x #angle of first arm in radians
        self.y = y  #angle of second arm in radians
        self.children = []    #all neighboring nodes
        self.parent = None     

#Represents the entire RRT tree
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
        if x == self.goal.x and y == self.goal.y:
            print("test")
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            tempNode = treeNode(x, y)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
    
    #Samples a random point between -pi and pi. Occasionally samples the goal node with a 5% probability
    def samplePoint(self, goal):
        x = random.uniform(-pi, pi)
        y = random.uniform(-pi, pi)
        prob = random.uniform(0, 1)
        if prob < 0.9:
            return np.array([x,y])
        else:
            return np.array(goal)
    
    #Travels the direction of the sampled node with the given step size
    def goToPoint(self, start, end):
        offset = self.rho*self.unitVector(start, end)
        point = np.array([start.x + offset[0], start.y + offset[1]])
        return point
    
    #checks whether the node on the graph corresponds to a collision in the arm.
    def isInObstacle(self, start, end, arm):
        u_hat = self.unitVector(start, end)
        test = np.array([0.0,0.0])
        for i in range(floor(self.rho/radians(5))):
            test[0] = start.x + i* u_hat[0]
            test[1] = start.y + i * u_hat[1]
            #check if arm in obstacle
            arm.theta1, arm.theta2 = test[0], test[1] 
            arm.re_orient() # Move robot to this configuration in workspace
            collisions = arm.check_arm_collisions() # Get boolean array of parts that collide
            if any(collision for collision in collisions): 
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
        dist = np.sqrt((node1.x - point[0])**2 + (node1.y - point[1])**2)
        return dist
    
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.rho:
            return True
    

    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDist = 10000

    def retracePath(self, goal):
        if goal.x == self.randomTree.x and goal.y == self.randomTree.y:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.x, goal.y])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance += self.rho
        self.retracePath(goal.parent)
        
    

def rtt_tree(start, goal,arm):
    plt.close('all')
    fig = plt.figure("RTT Algorithm")
    plt.plot(start[0], start[1], 'ro')
    ax = fig.gca()
    ax.set_xlim(-pi,pi)
    ax.set_ylim(-pi,pi)
    rrt = RTT(start, goal, 1000, radians(5))
    i = 0
    while i < 1000:
        rrt.resetNearestValues()
        point = rrt.samplePoint(goal)
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.goToPoint(rrt.nearestNode, point)
        bool = rrt.isInObstacle(rrt.nearestNode, new, arm)
        if bool == False:
            print("Iteration ", i)
            i += 1
            rrt.addChild(new[0], new[1])
            plt.pause(0.01)
            plt.plot([rrt.nearestNode.x, new[0]],[rrt.nearestNode.y, new[1]], 'go', linestyle="--")
            if rrt.goalFound(new):
                rrt.addChild(goal[0], goal[1])
                print("Goal Found")
                break
        elif i == 0: 
            print("Error: Start node in obstacle")
            break
    plt.pause(1)
    rrt.retracePath(rrt.goal)
    rrt.Waypoints.insert(0, start)
    for i in range(len(rrt.Waypoints) - 1):
        plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle="--")
    plt.pause(1)
    plt.close()
    fig,ax = plt.subplots(dpi=100)
    arm = Arm_Controller(start[0], start[1], ax)
    arm.set_arm_obs(poly_map)
    arm.set_obs_plot()
    arm.theta1, arm.theta2 = start
    arm_graph(start, arm, rrt.Waypoints)


def arm_graph(start,arm, waypoints):
    begin = start
    for i in range(len(waypoints) ):
        move_arm(arm, begin, waypoints[i])
        begin = waypoints[i]


def move_arm(arm, start, goal):
    discretized_pts = interpolate(start, (goal[0], goal[1]), radians(5))
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
    start1, start2 = angle_mod(args.start[0]), angle_mod(args.start[1])
    goal1, goal2 = angle_mod(args.goal[0]), angle_mod(args.goal[1])
    poly_map = load_polygons(args.map)
    
    planar_arm = Arm_Controller(start1, start2)
    planar_arm.set_arm_obs(poly_map)
    #Create the plot
    rtt_tree(np.array([start1, start2]), np.array([goal1, goal2]), planar_arm)
    #move_arm(planar_arm, (start1, start2), (goal1, goal2))
    '''
    tree = RTT((start1,start2))
    counter = 0
    planar_arm = Arm_Controller(args.start[0], args.start[1])
    planar_arm.set_arm_obs(poly_map)
    planar_arm.set_obs_plot()
    generate_sample(planar_arm, args.start, args.goal)
    '''


