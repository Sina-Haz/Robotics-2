import argparse
from planar_arm import Arm_Controller
from math import radians

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

# Usage: python3 arm_3.py --start 0 0 --goal 2 2
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='arm_3 will interpolate points from start to goal configurations and show the arm at these points')
    parser.add_argument('--start', nargs = 2, type=float, required=True, help='Starting configuration, give two floats in radians')
    parser.add_argument('--goal', nargs=2, type=float, required=True, help='Goal configuration, given as two floats in radians')
    args = parser.parse_args()

    print(interpolate(args.start, args.goal, radians(5)))

