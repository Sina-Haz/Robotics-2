import argparse
from arm import NLinkArm
from planar_arm import Arm_Controller
from math import radians
from create_scene import create_plot, show_scene
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

# Interpolate points on line segment from start to goal given some small resolution as a step size
def interpolate(start, goal, resolution):
    # Get equation for the line as such: y - y1 = m(x - x1), solve for m
    x1,y1 = start
    x2,y2 = goal
    slope = (y2-y1)/(x2-x1)
    points = []
    num_points = abs(int((x2-x1)/resolution + 1))
    xs = [x1 + i*resolution for i in range(num_points)]

    for x_i in xs:
        y_i = slope*(x_i - x1) + y1
        points.append((x_i, y_i))

    if points[-1] != goal:
        points.append(goal)
    return points

def update(frame, arm, points):
    if frame < len(points):
        arm.theta1, arm.theta2 = points[frame]
        arm.re_orient()
        arm.draw_arm()

# Usage: python3 arm_3.py --start 0 0 --goal 2 2
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='arm_3 will interpolate points from start to goal configurations and show the arm at these points')
    parser.add_argument('--start', nargs = 2, type=float, required=True, help='Starting configuration, give two floats in radians')
    parser.add_argument('--goal', nargs=2, type=float, required=True, help='Goal configuration, given as two floats in radians')
    args = parser.parse_args()
    discretized_pts = interpolate(args.start, args.goal, radians(5))

    # Code that works with the TAs arm.py file
    # # We discretize line segment in c-space according to resolution of 5 degrees
    # planar_arm = NLinkArm([0.3, 0.15], args.start, joint_radius=0.05, link_width=0.1)

    # for pt in discretized_pts:
    #     planar_arm.update_joints(pt) # Updates the joint angles and points
    #     planar_arm.plot() # Redraws the updated points
    # plt.close()
    # print('finished')

    planar_arm = Arm_Controller(args.start[0], args.start[1])
    planar_arm.draw_arm()
    for pt in discretized_pts:
        planar_arm.set_joint_angles(pt)
        planar_arm.re_orient()
        planar_arm.ax.cla()
        planar_arm.add_arm()
        planar_arm.ax.figure.canvas.draw()
        show_scene(planar_arm.ax)
    # plt.close()
    print('finished')








    

