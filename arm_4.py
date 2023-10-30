import argparse
from math import pi
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller
import numpy as np

class RTTree:
    def __init__(self, config):
        self.config = config
        self.parent = None
        self.nearest = []

if __name__=='__main__':
    # This code gets us the inputs from the command line
    parser = argparse.ArgumentParser(description="arm_2.py will find the two configurations in the file that are closest to the target")
    parser.add_argument('--start',  nargs=2, required=True, help='start orientation')
    parser.add_argument('--map', required=True, )
    parser.add_argument('--target',  nargs=2, required=True, help='target orientation')
    args = parser.parse_args()

    #get the parameters from the parser
    start1, start2 = args.start
    goal1, goal2 = args.goal
    poly_map = load_polygons(args.map)

    

    args = parser.parse_args()