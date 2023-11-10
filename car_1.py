import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import degrees, cos, sin, tan, pi
from create_scene import add_polygon_to_scene, create_plot, show_scene
import numpy as np
from rigid_body import check_boundary, check_car
from copy import deepcopy
import time

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
        self.run()

    # Add obstacles to the plot
    def set_obs_plot(self):
        for p in self.obs:
            add_polygon_to_scene(p,self.ax,False)

    # Computes q' based on current position and controls
    def get_q_delta(self):
        return [self.v*cos(self.theta), self.v*sin(self.theta), (self.v/self.L)*tan(self.phi)]
    
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

        self.x, self.y, self.theta = nextConfig
        self.update_body()

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
  
        
    def run(self):
        curr_position, currConfig = deepcopy(self.body), (self.x, self.y,self.theta) # Keep the current position in case we need it
        while True:
            self.compute_next_position()
            # If car goes out of bounds or hits something we should go back to prev position
            if (check_boundary(self.body) and check_car(self.body, self.obs)):
                self.body = curr_position
                self.x, self.y, self.theta = currConfig
            
            # Update the car's position
            self.fig.canvas.draw()




if __name__ == '__main__':
    fig = plt.figure("Car")
    dynamic_car = Car(ax=fig.gca(), startConfig=(0.5, 0.5, 0.5), dt = 0.1)
    show_scene(dynamic_car.ax)




        

        








        
    
    
