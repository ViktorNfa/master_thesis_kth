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
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib import animation
from scipy.optimize import minimize, LinearConstraint
from tqdm import tqdm

from distributed_auxiliary import *

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

# Robot size/radius (modelled as a circle with a directional arrow)
r_robot = 0.5

# Frequency of update of the simulation (in Hz)
freq = 50

# Maximum time of the simulation (in seconds)
max_T = 60

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

# CBF Communication maintenance or obstacle avoidance activation (1 is cm/-1 is oa)
cbf = -1

# Safe distance for communication maintenance and obstacle avoidance
d = 1.5

# Linear alpha function with parameter
alpha = 1

# Exponential parameter
p = 1

# Adaptative law parameter
k0 = 1

# Variable to determine if HuIL is active or not 
# (1 is activated/0 is deactivated) as well as the robot it affects
huil = 0
human_robot = number_robots

# HuIL parameters
v_huil = 8
division = 6


## Pre-calculations needed for controller and simulation

# Get the number of neighbours for each robot
number_neighbours = []
# Create edge list
edges = []
# Create Laplacian matrix for the graph
L_G = np.zeros((number_robots,number_robots))
# Create robot/wedge/extra_robot list for the name of columns
robot_col = ['Time']
# Setup controller output init output
controller = [0.]
nom_controller = [0.]
for i in range(number_robots):
    number_neighbours.append(len(neighbours[i]))
    L_G[i, i] = number_neighbours[i]
    robot_col.append("Robot_x"+str(i+1))
    robot_col.append("Robot_y"+str(i+1))
    for j in neighbours[i]:
        L_G[i, j-1] = -1
        if (i+1,j) not in edges and (j,i+1) not in edges:
            edges.append((i+1,j))
    controller.append(0.)
    controller.append(0.)
    nom_controller.append(0.)
    nom_controller.append(0.)

# Create edge list for the name of columns
edges_col = ['Time']
# Setup cbf functions init output
cbf_cmoa = [0.]
for i in range(len(edges)):
    edges_col.append("Edge"+str(edges[i]))
    cbf_cmoa.append(0.)

# Modify ideal formation positions to one column vector
x_d = np.reshape(formation_positions,number_robots*dim)


## Initialize logging

#Create dataframes to pandas the data
global df_cbf_cmoa
df_cbf = pd.DataFrame(columns=edges_col)
global df_controller
df_controller = pd.DataFrame(columns=robot_col)
global df_nom_controller
df_nom_controller = pd.DataFrame(columns=robot_col)  
global df_huil_controller
df_huil_controller = pd.DataFrame(columns=[robot_col[0], robot_col[2*human_robot-1], robot_col[2*human_robot]]) 


## Simulation and visualization loop

max_time_size = max_T*freq

# Initialize position matrix
x = np.zeros((number_robots*dim,max_time_size))

# Randomize initial position
x[:,0] = x_max*np.random.rand(number_robots*dim)-x_max/2

# Initialize slack variable
y = np.zeros((number_robots,max_time_size))

# Start simulation loop
print("Computing evolution of the system...")
for t in tqdm(range(max_time_size-1)):
    secs = t/freq

    # Compute nominal controller - Distributed
    u_nom = np.zeros((dim*number_robots, 1))
    for i in range(number_robots):
        u_nom[2*i:2*i+2] = formationController(i, number_neighbours[i], x[:,t], x_d)

    u_n = huilController(u_nom, huil, human_robot, t, max_time_size, v_huil, division)

    # Compute CBF constrained controller - Distributed
    u = np.zeros((dim*number_robots, 1))
    c = np.zeros((number_robots, 1))
    a = np.zeros((dim*number_robots, 1))
    b = np.zeros((number_robots, 1))
    for i in range(number_robots):

        for e in range(len(edges)):
            aux_i = edges[e][0]-1
            aux_j = edges[e][1]-1
            x_i = np.array([x[2*aux_i,t],x[2*aux_i+1,t]])
            x_j = np.array([x[2*aux_j,t],x[2*aux_j+1,t]])

            if i == aux_i:
                a[2*i:2*i+2] += -p*np.exp(-p*cbf_h(x_i, x_j, d, cbf))*cbf_gradh(x_i, x_j, cbf)
                b[i] += -alpha/2*(1/len(edges)-np.exp(-p*cbf_h(x_i, x_j, d, cbf)))
            elif i == aux_j:
                a[2*i:2*i+2] += -p*np.exp(-p*cbf_h(x_i, x_j, d, cbf))*cbf_gradh(x_j, x_i, cbf)
                b[i] += -alpha/2*(1/len(edges)-np.exp(-p*cbf_h(x_i, x_j, d, cbf)))
            else:
                a[2*i:2*i+2] += np.zeros((dim, 1))
                b[i] += 0

        c[i] = (np.dot(L_G[i,:],y[:,t])+np.dot(np.transpose(a[2*i:2*i+1]),u_n[2*i:2*i+1])+b[i])/np.dot(np.transpose(a[2*i:2*i+1]),a[2*i:2*i+1])
        u[2*i:2*i+1] = u_n[2*i:2*i+1] - np.maximum(0,c[i])*a[2*i:2*i+1]

    # Update slack variable
    y[:,t+1] = y[:,t] - np.transpose(k0*np.sign(np.dot(L_G,c)))*(1/freq)

    # Update the system using dynamics
    xdot = systemDynamics(x[:,t], u)
    x[:,t+1] = np.transpose(xdot)*(1/freq) + x[:,t]


    # Save data in dataframe
    # CBF functions
    cbf_cmoa[0] = secs
    for e in range(len(edges)):
        aux_i = edges[e][0]-1
        aux_j = edges[e][1]-1
        x_i = np.array([x[2*aux_i,t],x[2*aux_i+1,t]])
        x_j = np.array([x[2*aux_j,t],x[2*aux_j+1,t]])
        cbf_cmoa[e+1] = cbf_h(x_i, x_j, d, cbf)

    df2_cbf = pd.DataFrame(np.array([cbf_cmoa]), columns=edges_col)
    df_cbf = df_cbf.append(df2_cbf, ignore_index=True)
    
    # Final controller
    controller[0] = secs
    controller[1:] = u
    df2_controller = pd.DataFrame(np.array([controller]), columns=robot_col)
    df_controller = df_controller.append(df2_controller, ignore_index=True)

    # Nominal controller
    nom_controller[0] = secs
    nom_controller[1:] = u_nom
    df2_nom_controller = pd.DataFrame(np.array([nom_controller]), columns=robot_col)
    df_nom_controller = df_nom_controller.append(df2_nom_controller, ignore_index=True)

    # HuIL controller
    df2_huil_controller = pd.DataFrame(np.array([[secs, u_n[2*human_robot-2], u_n[2*human_robot-1]]]), columns=[robot_col[0], robot_col[2*human_robot-1], robot_col[2*human_robot]])
    df_huil_controller = df_huil_controller.append(df2_huil_controller, ignore_index=True)


## Visualize CBF conditions/plots & trajectories

print("Showing CBF function evolution...")

cbf_col = df_cbf.columns.values
starting_point = df_cbf[cbf_col[1]].ne(0).idxmax()

fig_cbf, ax_cbf = plt.subplots()  # Create a figure and an axes.
for i in range(len(cbf_col)):
    if i > 0:
        ax_cbf.plot(df_cbf[cbf_col[0]].iloc[starting_point:-1], df_cbf[cbf_col[i]].iloc[starting_point:-1], label=cbf_col[i])  # Plot some data on the axes.

if cbf == 1:
    # Plot the CBF comunication maintenance
    ax_cbf.set_xlabel('time')  # Add an x-label to the axes.
    ax_cbf.set_ylabel('h_cm')  # Add a y-label to the axes.
    ax_cbf.set_title("CBF functions for comunication maintenance")  # Add a title to the axes.
    ax_cbf.legend()  # Add a legend.
    ax_cbf.axhline(y=0, color='k', lw=1)

if cbf == -1:
    # Plot the CBF obstacle avoidance
    ax_cbf.set_xlabel('time')  # Add an x-label to the axes.
    ax_cbf.set_ylabel('h_oa')  # Add a y-label to the axes.
    ax_cbf.set_title("CBF functions for obstacle avoidance")  # Add a title to the axes.
    ax_cbf.legend()  # Add a legend.
    ax_cbf.axhline(y=0, color='k', lw=1)

# Plot the normed difference between nominal and final controller
controller_col = df_controller.columns.values
fig_norm, ax_norm = plt.subplots()  # Create a figure and an axes.
step = 1
ax_norm.axis('on')
for i in range(1, len(controller_col), 2):
    if i > 0:
        diff_x = df_controller[controller_col[i]].iloc[starting_point:-1] - df_nom_controller[controller_col[i]].iloc[starting_point:-1]
        diff_y = df_controller[controller_col[i+1]].iloc[starting_point:-1] - df_nom_controller[controller_col[i+1]].iloc[starting_point:-1]
        diff = np.array([diff_x, diff_y])
        normed_difference = np.sqrt(np.square(diff).sum(axis=0))
        ax_norm.plot(df_controller[controller_col[0]].iloc[starting_point:-1], normed_difference, label="Robot"+str(step))  # Plot some data on the axes.
        step += 1

diff_x = df_controller[controller_col[2*human_robot-1]].iloc[starting_point:-1] - df_huil_controller[controller_col[2*human_robot-1]].iloc[starting_point:-1]
diff_y = df_controller[controller_col[2*human_robot]].iloc[starting_point:-1] - df_huil_controller[controller_col[2*human_robot]].iloc[starting_point:-1]
diff = np.array([diff_x, diff_y])
normed_difference = np.sqrt(np.square(diff).sum(axis=0))
ax_norm.plot(df_controller[controller_col[0]].iloc[starting_point:-1], normed_difference, label="HuILDiff"+str(human_robot))  # Plot some data on the axes.

huil_x = df_huil_controller[controller_col[2*human_robot-1]].iloc[starting_point:-1]
huil_y = df_huil_controller[controller_col[2*human_robot]].iloc[starting_point:-1]
huil = np.array([huil_x, huil_y])
normed_huil = np.sqrt(np.square(huil).sum(axis=0))
#ax_norm.plot(df_controller[controller_col[0]].iloc[starting_point:-1], normed_huil, label="HuIL"+str(human_robot))  # Plot some data on the axes.

ax_norm.set_xlabel('time')  # Add an x-label to the axes.
ax_norm.set_ylabel('|u - u_nom|')  # Add a y-label to the axes.
ax_norm.set_title("Normed difference between u and nominal u")  # Add a title to the axes.
ax_norm.legend()  # Add a legend.
ax_norm.axhline(y=0, color='k', lw=1)

plt.show()


print("Showing animation...")

# Start figure and axes with limits
fig = plt.figure()
ax = plt.axes(xlim=(-winx, winx), ylim=(-winy, winy))

time_txt = ax.text(0.475, 0.975,'',horizontalalignment='left',verticalalignment='top', transform=ax.transAxes)

shapes = []
for i in range(number_robots):
    shapes.append(plt.Circle((x[2*i,0], x[2*i+1,0]), r_robot, fc='b'))

for i in range(len(edges)):
    aux_i = edges[i][0]-1
    aux_j = edges[i][1]-1
    shapes.append(plt.Line2D((x[2*aux_i,0], x[2*aux_j,0]), (x[2*aux_i+1,0], x[2*aux_j+1,0]), lw=0.5, color='b', alpha=0.3))

def init():
    for i in range(number_robots):
        shapes[i].center = (x[2*i,0], x[2*i+1,0])
        ax.add_patch(shapes[i])

    for i in range(len(edges)):
        aux_i = edges[i][0]-1
        aux_j = edges[i][1]-1
        shapes[number_robots+i].set_xdata((x[2*aux_i,0], x[2*aux_j,0]))
        shapes[number_robots+i].set_ydata((x[2*aux_i+1,0], x[2*aux_j+1,0]))
        ax.add_line(shapes[number_robots+i])

    time_txt.set_text('T=0.0 s')

    return shapes + [time_txt,]

def animate(frame):

    for i in range(number_robots):
        shapes[i].center = (x[2*i,frame], x[2*i+1,frame])

    for i in range(len(edges)):
        aux_i = edges[i][0]-1
        aux_j = edges[i][1]-1
        shapes[number_robots+i].set_xdata((x[2*aux_i,frame], x[2*aux_j,frame]))
        shapes[number_robots+i].set_ydata((x[2*aux_i+1,frame], x[2*aux_j+1,frame]))

    secs = frame/freq
    time_txt.set_text('T=%.1d s' % secs)

    return shapes + [time_txt,]

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=max_time_size, 
                               interval=1/freq*1000,
                               blit=True,
                               repeat=False)

plt.show()

#anim.save('animation.mp4', fps=50, extra_args=['-vcodec', 'libx264'])

print("Completed!")