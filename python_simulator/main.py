#!/usr/bin/env python
#=====================================
#          Python simulator
#          mobile 2D robots
#     Victor Nan Fernandez-Ayala 
#           (vnfa@kth.se)
#=====================================

# To force int division to floats (for Python 2.7)
from __future__ import division

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from scipy.optimize import minimize, LinearConstraint

from auxiliary import *

#plt.style.use("seaborn-whitegrid")


## Parameter setup

# Dimensionality of the problem
dim = 2

# Window size
winx = 30
winy = 30

# Arena size
x_max = winx-5
y_max = winy-5

# Robot size/diameter (modelled as a circle with a directional arrow)
d_robot = 0.5

# Frequency of update of the simulation (in Hz)
freq = 50

# Maximum time of the simulation (in seconds)
max_T = 100

# Ideal formation positions
formation_positions = [[0, 2], [0, 0], [0, -2], [2, 2], [2, -2]]
#formation_positions = [[0, 10], [0, 8], [0, 6], [0, 4], [0, 2], [0, 0], [0, -2], [0, -4], [0, -6], [0, -8], [0, -10], 
#                        [10, 10], [8, 8], [6, 6], [4, 4], [2, 2], [2, -2], [4, -4], [6, -6], [8, -8], [10, -10]]

# Get the number of robots
number_robots = len(formation_positions)

# List of neighbours for each robot
neighbours = [[2], [1, 3, 4, 5], [2], [2], [2]]
#neighbours = [[2, 4], [1, 3, 4, 5], [2, 5], [1, 2], [2, 3]]
#neighbours = [[2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7, 16, 17], [6, 8], [7, 9], [8, 10], [9, 11], [10], 
#               [13], [12, 14], [13, 15], [14, 16], [6, 15], [6, 18], [17, 19], [18, 20], [19, 21], [20]]
#neighbours = [[i+1 for i in range(number_robots) if i != j] for j in range(number_robots)]

# CBF Communication maintenance or obstacle avoidance activation 
# (1 is activated/0 is deactivated)
cm = 0
oa = 1

# Safe distance for communication maintenance and obstacle avoidance
d_cm = 3
d_oa = 1.1

# Linear alpha function with parameter
alpha = 1

# Variable to determine if HuIL is active or not 
# (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1
human_robot = 5
#human_robot = 21


## Pre-calculations needed for controller and simulation

# Get the number of neighbours for each robot
number_neighbours = []
for i in range(number_robots):
    number_neighbours.append(len(neighbours[i]))

# Modify ideal formation positions to one column vector
p_d = np.reshape(formation_positions,number_robots*dim)

#Create Laplacian matrix for the graph
L_G = np.zeros((number_robots,number_robots))
for i in range(number_robots):
    L_G[i, i] = number_neighbours[i]
    for j in neighbours[i]:
        L_G[i, j-1] = -1

#Create edge list
edges = []
for i in range(number_robots):
    for j in neighbours[i]:
        if (i+1,j) not in edges and (j,i+1) not in edges:
            edges.append((i+1,j))

# Create safety constraint for arena
As = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
A_arena = np.zeros((number_robots*4, number_robots*2))
for i in range(number_robots):
    A_arena[4*i:4*i+4, 2*i:2*i+2] = As
b_arena = np.zeros((number_robots*4))

#Create wedge CBF
Aw = np.array([[-y_max/(2*x_max), -1], [-y_max/(2*x_max), 1]])
A_wedge = np.zeros((number_robots*2, number_robots*2))
for i in range(number_robots):
    A_wedge[2*i:2*i+2, 2*i:2*i+2] = Aw
b_wedge = np.zeros((number_robots*2))


## Simulation and visualization loop

max_time_size = max_T*freq

# Initialize position matrix
p = np.zeros((number_robots*dim,max_time_size))

# Randomize initial position
p[:,0] = x_max*np.random.rand(number_robots*dim)-x_max/2

# Start simulation loop
for i in range(max_time_size-1):
    # Compute nominal controller - Centralized and Distributed
    u_nom = formationController(L_G, p[:,i], p_d)

    # Add HuIL controll
    u_n = huilController(u_nom, huil, human_robot, i, max_time_size)

    # Compute CBF constrained controller (w and w/out arena safety, wedge shape or extra robot) - Centralized and Distributed
    #u = cbfController(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha)
    #u = cbfControllerWArena(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha, A_arena, b_arena, x_max, -x_max, y_max, -y_max)
    u = cbfControllerWArenaWedge(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha, A_arena, b_arena, x_max, -x_max, y_max, -y_max, A_wedge, b_wedge)

    # Update the system using dynamics
    pdot = systemDynamics(p[:,i], u)
    p[:,i+1] = pdot*(1/freq) + p[:,i]
    

## Visualize trajectories

# Start figure and axes with limits
fig = plt.figure()
ax = plt.axes(xlim=(-winx, winx), ylim=(-winy, winy))

# Add the limits of the arena
arena_limit1 = plt.Line2D((-x_max, x_max), (y_max, y_max), lw=2.5, color='r')
arena_limit2 = plt.Line2D((-x_max, x_max), (-y_max, -y_max), lw=2.5, color='r')
arena_limit3 = plt.Line2D((x_max, x_max), (-y_max, y_max), lw=2.5, color='r')
arena_limit4 = plt.Line2D((-x_max, -x_max), (-y_max, y_max), lw=2.5, color='r')
plt.gca().add_line(arena_limit1)
plt.gca().add_line(arena_limit2)
plt.gca().add_line(arena_limit3)
plt.gca().add_line(arena_limit4)

# Add wedge limits (OPTIONAL)
wedge1 = plt.Line2D((-x_max, x_max), (y_max, 0), lw=1, color='r', alpha=0.7)
wedge2 = plt.Line2D((-x_max, x_max), (-y_max, 0), lw=1, color='r', alpha=0.7)
plt.gca().add_line(wedge1)
plt.gca().add_line(wedge2)

shapes = []
for i in range(number_robots):
    shapes.append(plt.Circle((p[2*i,0], p[2*i+1,0]), d_robot, fc='b'))

for i in range(len(edges)):
    aux_i = edges[i][0]-1
    aux_j = edges[i][1]-1
    shapes.append(plt.Line2D((p[2*aux_i,0], p[2*aux_j,0]), (p[2*aux_i+1,0], p[2*aux_j+1,0]), lw=0.5, color='b', alpha=0.3))

def init():
    for i in range(number_robots):
        shapes[i].center = (p[2*i,0], p[2*i+1,0])
        ax.add_patch(shapes[i])

    for i in range(len(edges)):
        aux_i = edges[i][0]-1
        aux_j = edges[i][1]-1
        shapes[number_robots+i].set_xdata((p[2*aux_i,0], p[2*aux_j,0]))
        shapes[number_robots+i].set_ydata((p[2*aux_i+1,0], p[2*aux_j+1,0]))
        ax.add_line(shapes[number_robots+i])

    return shapes

def animate(frame):
    for i in range(number_robots):
        shapes[i].center = (p[2*i,frame], p[2*i+1,frame])

    for i in range(len(edges)):
        aux_i = edges[i][0]-1
        aux_j = edges[i][1]-1
        shapes[number_robots+i].set_xdata((p[2*aux_i,frame], p[2*aux_j,frame]))
        shapes[number_robots+i].set_ydata((p[2*aux_i+1,frame], p[2*aux_j+1,frame]))

    return shapes

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=max_time_size, 
                               interval=1/freq*1000,
                               blit=True,
                               repeat=False)

plt.show()

