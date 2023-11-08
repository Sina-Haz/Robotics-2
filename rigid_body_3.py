import argparse
from math import sqrt, pi, degrees, radians, atan
from rigid_body import CarController
from create_scene import create_plot, show_scene
from rigid_body_1 import make_rigid_body
from planar_arm import angle_mod
import matplotlib.pyplot as plt

def get_slope_vector(pt1, pt2):
    direction = [x2 - x1 for x1,x2 in zip(pt1, pt2)]
    magnitude = sqrt(sum(x**2 for x in direction))
    return tuple(x/magnitude for x in direction)

def reposition_car(config, car):
    x,y,theta = config
    new_car = make_rigid_body((x,y))
    car.car.set_x(new_car.get_x())
    car.car.set_y(new_car.get_y())
    car.car.set_angle(degrees(theta))



def interpolate(start, goal, resP = 0.05, resA = radians(10)):
    x1,y1,t1 = start
    x2,y2,t2 = goal
    euclidean_dist = sqrt((x2-x1)**2 + (y2-y1)**2)  #Find Euclidean distance
    points = []
    angle = atan((y2-y1)/(x2-x1))       #Find angle between start and goal nodes
    num_angles = int((angle-t1)/resA)    #Set the rigid body to that angle before moving
    for i in range(num_angles):
        points.append((x1,y1,t1+resA))
    points.append((x1,y1,angle))

    num_points = int(euclidean_dist / resP)
    for i in range(num_points+1):
        t = 0
        if num_points != 0:
            t = i/num_points
        x_i = (1-t)*x1 + t*x2
        y_i = (1-t)*y1 + t*y2
        points.append((x_i, y_i, angle))
    angle_to_goal = int((t2-angle)/resA)       #Find angle between that angle and goal
    for i in range(angle_to_goal):
        points.append((x2,y2,angle+resA))
    points.append((x2, y2, t2))
    return points


# Usage: python3 rigid_body_3.py --start 0.5 0.5 0 --goal 1.2 1.0 0.5
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='rigid_body_3 will interpolate points from start to goal configurations in SE(2), and animate an obstacle free workspace where we move along these points')
    parser.add_argument('--start', nargs = 3, type=float, required=True, help='Starting configuration, give 3 floats in radians, x and y and then a theta in range [-pi, pi]')
    parser.add_argument('--goal', nargs=3, type=float, required=True, help='Goal configuration, give 3 floats in radians, x and y and then a theta in range [-pi, pi]')
    args = parser.parse_args()

    discretized_pts = interpolate(args.start, args.goal, 0.05)
    plt.close('all')
    rig_body = CarController(ax = create_plot(), car = make_rigid_body((args.start[:2])), obstacles=[])
    rig_body.car.set_angle(degrees(args.start[2]))
    for pt in discretized_pts:
        reposition_car(pt, rig_body)
        rig_body.ax.cla()
        rig_body.ax.set_ylim([0,2])
        rig_body.ax.set_xlim([0,2])
        rig_body.ax.add_patch(rig_body.car)
        plt.draw()
        plt.pause(.1)
        rig_body.ax.figure.canvas.draw()
    print('finished')
    


