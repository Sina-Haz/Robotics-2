import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import degrees, cos, sin, tan, pi, radians
from create_scene import add_polygon_to_scene, create_plot, show_scene
import numpy as np
from matplotlib.animation import FuncAnimation
from rigid_body import CarController, check_boundary, check_car
from rigid_body_1 import make_rigid_body
from copy import deepcopy
import time
import argparse

class Car:
    def __init__(self, ax, startConfig, dt, width = 0.2, height = 0.1, obstacles = []):
        self.ax = ax
        self.width = width
        self.height = height
        self.x, self.y, self.theta = startConfig
        self.L = 0.2 # length of wheelbase
        self.obs = obstacles
        self.dt = dt

        # Initial control inputs are 0
        self.v, self.phi = 0,0
        self.continue_anim=True

        # Have car body reflect starting config
        self.body = patches.Rectangle((self.x, self.y), self.width, self.height)
        self.body.set_angle(degrees(self.theta))
        self.ax.add_patch(self.body)
        self.fig = ax.figure
         # Set the axis limits
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(0, 2)
        # Connect the event to the callback function
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    # Add obstacles to the plot
    def set_obs_plot(self):
        for p in self.obs:
            add_polygon_to_scene(p,self.ax,False)

    # Computes q' based on current position and controls
    def get_q_delta(self):
        return [self.v*cos(self.theta), self.v*sin(self.theta), (self.v/self.L)*tan(self.phi)]
    
    #Sets velocity and direction of car
    def set_velocity(self, v, phi):
        self.v = v
        self.phi = phi

    # Updates the body using current configuration
    def update_body(self):
        self.body.set_x(self.x)
        self.body.set_y(self.y)
        self.body.set_angle(degrees(self.theta))


    # Computes next configuration based on controls and dt, updates self.x, self.y, self.theta, and self.body
    def compute_next_position(self):
        currConfig = np.array([self.x, self.y, self.theta])
        q_delta = np.array(self.get_q_delta())
        nextConfig = currConfig + q_delta*self.dt
        placeholder_car = make_rigid_body(nextConfig[:2], nextConfig[2])
        if not collides_no_controller(placeholder_car, self.obs):
            self.x, self.y, self.theta = nextConfig
            self.update_body()
        else:
            # Set velocity to 0 and don't update the robots configuration or body
            self.update_velocity(0)

    # Update the velocity making sure to stay within the restraints of [-0.5, 0.5]
    def update_velocity(self, v):
        if v >= 0:
            self.v = min(0.5, v)
        elif v < 0:
            self.v = max(-0.5, v)
    
    # Update phi while staying within restraints.
    def update_phi(self, phi):
        if phi >= 0:
            self.phi = min(pi/4, phi)
        elif phi < 0:
            self.phi = max(-pi/4, phi)

    # Use arrow keys up and down for velocity, left and right for phi. This will run in a loop where each new frame changes
    # by dt.
    def on_key_press(self, event):
        
        vel_delta = 0.05
        phi_delta = pi/40
        if event.key == 'up':
            self.update_velocity(self.v + vel_delta)
        elif event.key == 'down':
            self.update_velocity(self.v - vel_delta)
        elif event.key == 'left':
            self.update_phi(self.phi + phi_delta)
        elif event.key == 'right':
            self.update_phi(self.phi - phi_delta)
        elif event.key == 'q':
            self.continue_anim=False
    

    # Add this method to initialize the animation
    def init_animation(self):
        return [self.body]

    # Add this method to update the animation at each frame
    def update_animation(self, frame):
        self.compute_next_position()
        return [self.body]


    # Add this method to start the animation loop
    def start_animation(self):
        animation = FuncAnimation(self.fig, self.update_animation, init_func=self.init_animation, blit=True)
        plt.show()


def collides_no_controller(car_body, obstacles):
    return not check_car(car_body, obstacles) and not check_boundary(car_body)
        


if __name__ == '__main__':
    fig = plt.figure("Car")
    parser = argparse.ArgumentParser(description="My Script")
    parser.add_argument("--myArg")
    args, leftovers = parser.parse_known_args()
    parser.add_argument('--control', type=float, nargs=2, required=False, help='control')
    parser.add_argument('--start', type=float, nargs=3, required=False, help='target orientation')
    args = parser.parse_args()
    if args is None:
        dynamic_car = Car(ax=fig.gca(), startConfig=(0.5, 0.5, 0.5), dt = 0.1)
        dynamic_car.start_animation()
    else:
        print("test")
        v,phi = args.control
        dynamic_car = Car(ax=fig.gca(), startConfig = args.start,dt = 0.1)
        dynamic_car.set_velocity(v,phi)
        dynamic_car.start_animation()
