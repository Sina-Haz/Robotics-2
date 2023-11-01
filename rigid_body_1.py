# Basically we want to be able to run the command: python arm_1.py --map "some_map.npy" and it should draw a
# plot of the planar arm on the specified map and have it at a random collision free configuration
import argparse
from math import pi
import random
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller
import matplotlib.pyplot as plt
from rigid_body import CarController, check_car_spawn

def get_sample(arm):
    def sample():
        theta1 = random.random()*2*pi
        theta2 = random.random()*2*pi
        return theta1,theta2
    while True:
        arm.theta1, arm.theta2 = sample() # Get a sample
        arm.re_orient() # Move robot to this configuration in workspace
        collisions = arm.check_arm_collisions() # Get boolean array of parts that collide
        if all(not collision for collision in collisions): break
    

    

# Run the following command for output: python rigid_body_1.py --map "rigid_polygons.npy"
if __name__=='__main__':
    # This code just gets us the name of the map from the command line
    parser = argparse.ArgumentParser(description="car_1.py will draw a random collision free configuration of the robot given a workspace")
    parser.add_argument('--map', required=True, help='Path to the map file (e.g., "arm_polygons.npy")')
    args = parser.parse_args()
    poly_map = load_polygons(args.map)
    ax = create_plot()
    for poly in poly_map:
        add_polygon_to_scene(poly, ax, True)
    car = check_car_spawn(poly_map)
    ax.add_patch(car)
    show_scene(ax)
    





