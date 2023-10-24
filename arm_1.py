# Basically we want to be able to run the command: python arm_1.py --map "some_map.npy" and it should draw a
# plot of the planar arm on the specified map and have it at a random collision free configuration

import argparse
from create_scene import create_plot, add_polygon_to_scene, load_polygons, show_scene
from planar_arm import Arm_Controller



if __name__=='__main__':
    # This code just gets us the name of the map from the command line
    parser = argparse.ArgumentParser(description="arm_1.py will draw a random collision free configuration of the robot given a workspace")
    parser.add_argument('--map', required=True, help='Path to the map file (e.g., "arm_polygons.npy")')
    args = parser.parse_args()
    poly_map = load_polygons(args.map)

    ax = create_plot()
    planar_arm = Arm_Controller(0,0,ax, polygons=poly_map)
    planar_arm.set_obs_plot()
    planar_arm.draw_arm()

    





