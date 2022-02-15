#!/usr/bin/env python
#=====================================
#          Python simulator
#          mobile 2D robots
#     Victor Nan Fernandez-Ayala 
#           (vnfa@kth.se)
#=====================================

from re import A
import matplotlib.pyplot as plt 
import numpy as np
import time

from auxiliary import *


## Parameter setup

# Dimensionality of the problem
dim = 2

# Arena size
x_max = 20
y_max = 20

# Robot size/diameter (modelled as a circle with a directional arrow)
d_robot = 1

# Frequency of update of the simulation (in Hz)
freq = 50

# Maximum time of the simulation (in seconds)
max_T = 30

# Ideal formation positions
formation_positions = [[0, 2], [0, 0], [0, -2], [2, 2], [2, -2]]

# List of neighbours for each robot
neighbours = [[2], [1, 3, 4, 5], [2], [2], [2]]

# CBF Communication maintenance or obstacle avoidance activation 
# (1 is activated/0 is deactivated)
cm = 1
oa = 1

# Safe distance for communication maintenance and obstacle avoidance
d_cm = 3
d_oa = 0.9

# Variable to determine if HuIL is active or not 
# (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1
human_robot = 5


## Pre-calculations needed for controller and simulation

# Get the number of robots
number_robots = len(neighbours)

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


## Simulation and visualization loop

max_time_size = max_T*freq

# Initialize position matrix
p = np.zeros((number_robots*dim,max_time_size))

# Randomize initial position
p[:,0] = x_max*2*np.random.rand(number_robots*dim)-x_max

# Start simulation loop
for i in range(max_time_size-1):
    # Compute nominal controller
    u_nom = formationController(L_G, p[:,i], p_d)

    # Add HuIL controll
    u = huilController(u_nom, huil, human_robot, i, max_time_size)

    # Compute CBF constrained controller
    u = cbfController(p[:,i], u, cm, oa, d_cm, d_oa, edges)

    # Update the system using dynamics
    pdot = systemDynamics(p[:,i], u)
    p[:,i+1] = pdot*(1/freq) + p[:,i]
    

## Visualize trajectories


