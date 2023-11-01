import argparse
from math import pi, radians
import random
from arm import NLinkArm
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from arm_1 import get_sample
from arm_3 import interpolate
from planar_arm import Arm_Controller
import numpy as np
import matplotlib.pyplot as plt

class RTTree:
    def __init__(self, config):
        self.config = config
        self.parent = None
        self.nearest = []


def generate_sample(arm, start, goal):
    theta1 = random.random()*2*pi
    theta2 = random.random()*2*pi
    discretized_pts = interpolate(start, (theta1, theta2), radians(5))
    for pt in discretized_pts:
        arm.update_joints(pt) # Updates the joint angles and points
        arm.plot() # Redraws the updated points
    

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
    
    tree = RTTree((start1,start2))
    counter = 0
    planar_arm = NLinkArm([0.3, 0.15], args.start, joint_radius=0.05, link_width=0.1)
    for poly in poly_map:
        planar_arm.ax.add_patch(plt.Polygon(poly, closed = True, fill=True,color = 'black',alpha = 0.4))
        plt.plot()
    planar_arm.set_arm_obs(poly_map)
    generate_sample(planar_arm, args.start, args.goal)


