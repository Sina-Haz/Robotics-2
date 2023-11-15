import argparse
from arm import NLinkArm
from planar_arm import Arm_Controller
from math import radians
from create_scene import create_plot
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from math import pi

def get_theta_in_range(theta):
    # Use wrap around to keep theta in range [-pi, pi]
    if theta > pi:
        theta -= 2*pi
    elif theta < -pi:
        theta += 2*pi
    return theta

def interpolate(start, goal, resolution):
    x1, y1 = start
    x2, y2 = goal

    # Calculate the angular difference between start and goal
    angle_diff = get_theta_in_range(y2 - y1)

    # Determine the sign of rotation (clockwise or counterclockwise)
    sign = 1 if angle_diff >= 0 else -1

    # Calculate the total number of steps needed
    num_steps = int(abs(angle_diff) / resolution) + 1

    # Calculate the angular step size
    angle_step = angle_diff / num_steps

    # Initialize the list of points
    points = [start]

    # Generate interpolated points
    for i in range(1, num_steps):
        x_i = get_theta_in_range(x1 + i * resolution)
        y_i = get_theta_in_range(y1 + i * angle_step * sign)
        points.append((x_i, y_i))

    # Ensure that the last point is the goal
    if points[-1] != goal:
        points.append(goal)

    return points




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
    # planar_arm.draw_arm()
    # for pt in discretized_pts:
    #     print(pt)
    #     planar_arm.set_joint_angles(pt)
    #     planar_arm.re_orient()
    #     planar_arm.ax.cla()
    #     planar_arm.draw_arm()
    #     planar_arm.ax.figure.canvas.draw()
    # # plt.close()

    # Set up the figure and axis for animation
    # fig, ax = plt.subplots()
    # ax.set_aspect('equal', 'box')  # Adjust as needed

    # Function to update the animation at each frame
    def update(frame):
        planar_arm.ax.cla()
        joint_angles = discretized_pts[frame]
        planar_arm.set_joint_angles(joint_angles)
        planar_arm.re_orient()
        planar_arm.draw_arm()

    # Create the animation
    anim = FuncAnimation(planar_arm.ax.figure, update, frames=len(discretized_pts), interval=100, repeat=False)
    # anim.save('videos/arm1.3-3.gif', writer='pillow', fps=30)

    # Display the animation
    plt.show()
    print('finished')