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

from auxiliary import *

#plt.style.use("seaborn-whitegrid")


## Parameter setup

# Dimensionality of the problem
dim = 2

# Window size
winx = 20
winy = 20

# Arena size
x_max = winx-5
y_max = winy-5

# Robot size/radius (modelled as a circle with a directional arrow)
r_robot = 0.5

# Frequency of update of the simulation (in Hz)
freq = 100

# Maximum time of the simulation (in seconds)
max_T = 30

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
cm = 1
oa = 0

# Safe distance for communication maintenance and obstacle avoidance
d_cm = 2
d_oa = 1.1

# Linear alpha function with parameter
alpha = 1

# Variable to determine if HuIL is active or not 
# (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1
human_robot = number_robots

# HuIL parameters
v_huil = 2
division = 6

# Parameter to decide if wedge is shown or not
wedge = False

# Parameter to decide if Extra robot is shown or not
extra_robot = False

# Safety check, if wedge or extra_robot is active. cm should not be
if wedge or extra_robot:
    cm = 0


## Pre-calculations needed for controller and simulation

# Get the number of neighbours for each robot
number_neighbours = []
# Create edge list
edges = []
# Create Laplacian matrix for the graph
L_G = np.zeros((number_robots,number_robots))
# Create robot/wedge/extra_robot list for the name of columns
robot_col = ['Time']
wedge_col = ['Time']
extra_robot_col = ['Time']
# Setup controller output init output
controller = [0.]
nom_controller = [0.]
cbf_extra_robot = [0.]
cbf_wedge = [0.]
for i in range(number_robots):
    number_neighbours.append(len(neighbours[i]))
    L_G[i, i] = number_neighbours[i]
    robot_col.append("Robot_x"+str(i+1))
    robot_col.append("Robot_y"+str(i+1))
    wedge_col.append("Robot_Up"+str(i+1))
    wedge_col.append("Robot_Low"+str(i+1))
    extra_robot_col.append("Robot"+str(i+1))
    for j in neighbours[i]:
        L_G[i, j-1] = -1
        if (i+1,j) not in edges and (j,i+1) not in edges:
            edges.append((i+1,j))
    controller.append(0.)
    controller.append(0.)
    nom_controller.append(0.)
    nom_controller.append(0.)
    cbf_extra_robot.append(0.)
    cbf_wedge.append(0.)

# Create edge list for the name of columns
edges_col = ['Time']
# Setup cbf functions init output
cbf_cm = [0.]
cbf_oa = [0.]
for i in range(len(edges)):
    edges_col.append("Edge"+str(edges[i]))
    cbf_cm.append(0.)
    cbf_oa.append(0.)

# Modify ideal formation positions to one column vector
p_d = np.reshape(formation_positions,number_robots*dim)

# Create safety constraint for arena
As = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
A_arena = np.zeros((number_robots*4, number_robots*2))
# Create safety constraint for wedge shape
Aw = np.array([[-y_max/(2*x_max), -1], [-y_max/(2*x_max), 1]])
A_wedge = np.zeros((number_robots*2, number_robots*2))
for i in range(number_robots):
    A_arena[4*i:4*i+4, 2*i:2*i+2] = As
    A_wedge[2*i:2*i+2, 2*i:2*i+2] = Aw
b_arena = np.zeros((number_robots*4)) 
b_wedge = np.zeros((number_robots*2))

# Create safety constraint for extra robot
A_extra = np.zeros((number_robots, number_robots*dim))
b_extra = np.zeros((number_robots))


## Initialize logging

#Create dataframes to pandas the data
global df_cbf_cm
df_cbf_cm = pd.DataFrame(columns=edges_col)
global df_cbf_oa
df_cbf_oa = pd.DataFrame(columns=edges_col)
global df_controller
df_controller = pd.DataFrame(columns=robot_col)
global df_nom_controller
df_nom_controller = pd.DataFrame(columns=robot_col)  
global df_huil_controller
df_huil_controller = pd.DataFrame(columns=[robot_col[0], robot_col[2*human_robot-1], robot_col[2*human_robot]]) 
global df_cbf_wedge
df_cbf_wedge = pd.DataFrame(columns=wedge_col)
global df_cbf_extra_robot
df_cbf_extra_robot = pd.DataFrame(columns=extra_robot_col)  

## Simulation and visualization loop

max_time_size = max_T*freq

# Initialize position matrix
p = np.zeros((number_robots*dim,max_time_size))

# Initialize position for extra robot
huil_p = np.zeros((dim,max_time_size))

# Randomize initial position
p[:,0] = x_max*np.random.rand(number_robots*dim)-x_max/2

# Random initial position for extra robot
huil_p[:,0] = np.array([-5, 20])

# Start simulation loop
print("Computing evolution of the system...")
for i in tqdm(range(max_time_size-1)):
    secs = i/freq
    
    # Compute nominal controller - Centralized and Distributed
    u_nom = formationController(L_G, p[:,i], p_d)

    # Add HuIL control
    if extra_robot:
        u_n = u_nom
    else:
        u_n = huilController(u_nom, huil, human_robot, i, max_time_size, v_huil, division)

    # Compute CBF constrained controller (w and w/out arena safety, wedge shape or extra robot) - Centralized and Distributed
    if extra_robot:
        u, b_cm, b_oa, b_extra_robot = cbfControllerWArenaExtra(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha, A_arena, b_arena, x_max, -x_max, y_max, -y_max, A_extra, b_extra, d_oa, huil_p[:,i], v_huil, v_huil)
        # Save extra robot cbf in dataframe
        cbf_extra_robot[0] = secs
        cbfextra_robot = b_extra_robot/alpha
        cbf_extra_robot[1:] = cbfextra_robot.tolist()
        df2_cbf_extra_robot = pd.DataFrame(np.array([cbf_extra_robot]), columns=extra_robot_col)
        df_cbf_extra_robot = df_cbf_extra_robot.append(df2_cbf_extra_robot, ignore_index=True)
    elif wedge:
        u, b_cm, b_oa, b_wedgie = cbfControllerWArenaWedge(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha, A_arena, b_arena, x_max, -x_max, y_max, -y_max, A_wedge, b_wedge)
        # Save wedge cbf in dataframe
        cbf_wedge[0] = secs
        cbfwedge = b_wedgie/alpha
        cbf_wedge[1:] = cbfwedge.tolist()
        df2_cbf_wedge = pd.DataFrame(np.array([cbf_wedge]), columns=wedge_col)
        df_cbf_wedge = df_cbf_wedge.append(df2_cbf_wedge, ignore_index=True)
    else:
        u, b_cm, b_oa = cbfController(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha)
        #u, b_cm, b_oa = cbfControllerWArena(p[:,i], u_n, cm, oa, d_cm, d_oa, number_robots, edges, dim, alpha, A_arena, b_arena, x_max, -x_max, y_max, -y_max)

    # Update the system using dynamics
    pdot = systemDynamics(p[:,i], u)
    p[:,i+1] = pdot*(1/freq) + p[:,i]

    # Update extra robot (if applicable)
    if extra_robot:
        huil_pdot = extraRobotDynamics(i, max_time_size, v_huil, division)
        huil_p[:,i+1] = huil_pdot*(1/freq) + huil_p[:,i]

    # Save data in dataframe
    # CBF functions
    cbf_cm[0] = secs
    cbfcm = b_cm/alpha
    cbf_cm[1:] = cbfcm.tolist()
    df2_cbf_cm = pd.DataFrame(np.array([cbf_cm]), columns=edges_col)
    df_cbf_cm = df_cbf_cm.append(df2_cbf_cm, ignore_index=True)
    cbf_oa[0] = secs
    cbfoa = b_oa/alpha
    cbf_oa[1:] = cbfoa.tolist()
    df2_cbf_oa = pd.DataFrame(np.array([cbf_oa]), columns=edges_col)
    df_cbf_oa = df_cbf_oa.append(df2_cbf_oa, ignore_index=True)
    
    # Final controller
    controller[0] = secs
    controller[1:] = u.tolist()
    df2_controller = pd.DataFrame(np.array([controller]), columns=robot_col)
    df_controller = df_controller.append(df2_controller, ignore_index=True)

    # Nominal controller
    nom_controller[0] = secs
    nom_controller[1:] = u_nom.tolist()
    df2_nom_controller = pd.DataFrame(np.array([nom_controller]), columns=robot_col)
    df_nom_controller = df_nom_controller.append(df2_nom_controller, ignore_index=True)

    # HuIL controller
    df2_huil_controller = pd.DataFrame(np.array([[secs, u_n[2*human_robot-2], u_n[2*human_robot-1]]]), columns=[robot_col[0], robot_col[2*human_robot-1], robot_col[2*human_robot]])
    df_huil_controller = df_huil_controller.append(df2_huil_controller, ignore_index=True)


## Visualize CBF conditions/plots & trajectories

print("Showing CBF function evolution...")

cbf_cm_col = df_cbf_cm.columns.values
starting_point = df_cbf_cm[cbf_cm_col[1]].ne(0).idxmax()
if cm == 1:
    # Plot the CBF comunication maintenance
    fig_cbf_cm, ax_cbf_cm = plt.subplots()  # Create a figure and an axes.
    for i in range(len(cbf_cm_col)):
        if i > 0:
            ax_cbf_cm.plot(df_cbf_cm[cbf_cm_col[0]].iloc[starting_point:-1], df_cbf_cm[cbf_cm_col[i]].iloc[starting_point:-1], label=cbf_cm_col[i])  # Plot some data on the axes.

    ax_cbf_cm.set_xlabel('time')  # Add an x-label to the axes.
    ax_cbf_cm.set_ylabel('h_cm')  # Add a y-label to the axes.
    ax_cbf_cm.set_title("CBF functions for comunication maintenance")  # Add a title to the axes.
    ax_cbf_cm.legend()  # Add a legend.
    ax_cbf_cm.axhline(y=0, color='k', lw=1)

if oa == 1:
    # Plot the CBF obstacle avoidance
    cbf_oa_col = df_cbf_oa.columns.values
    fig_cbf_oa, ax_cbf_oa = plt.subplots()  # Create a figure and an axes.
    for i in range(len(cbf_oa_col)):
        if i > 0:
            ax_cbf_oa.plot(df_cbf_oa[cbf_oa_col[0]].iloc[starting_point:-1], df_cbf_oa[cbf_oa_col[i]].iloc[starting_point:-1], label=cbf_oa_col[i])  # Plot some data on the axes.

    ax_cbf_oa.set_xlabel('time')  # Add an x-label to the axes.
    ax_cbf_oa.set_ylabel('h_oa')  # Add a y-label to the axes.
    ax_cbf_oa.set_title("CBF functions for obstacle avoidance")  # Add a title to the axes.
    ax_cbf_oa.legend()  # Add a legend.
    ax_cbf_oa.axhline(y=0, color='k', lw=1)

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

if not extra_robot:
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

if wedge:
    # Plot the CBF wedge shape
    cbf_wedge_col = df_cbf_wedge.columns.values
    fig_cbf_wedge, ax_cbf_wedge = plt.subplots()  # Create a figure and an axes.
    ax_cbf_wedge.axis('on')
    for i in range(1, len(cbf_wedge_col)):
        if i > 0:
            ax_cbf_wedge.plot(df_cbf_wedge[cbf_wedge_col[0]].iloc[starting_point:-1], df_cbf_wedge[cbf_wedge_col[i]].iloc[starting_point:-1], label=cbf_wedge_col[i])  # Plot some data on the axes.

    ax_cbf_wedge.set_xlabel('time')  # Add an x-label to the axes.
    ax_cbf_wedge.set_ylabel('h_wedge')  # Add a y-label to the axes.
    ax_cbf_wedge.set_title("CBF functions for wedge shape")  # Add a title to the axes.
    ax_cbf_wedge.legend()  # Add a legend.
    ax_cbf_wedge.axhline(y=0, color='k', lw=1)

if extra_robot:
    # Plot the CBF extra robot
    cbf_extra_robot_col = df_cbf_extra_robot.columns.values
    fig_cbf_extra_robot, ax_cbf_extra_robot = plt.subplots()  # Create a figure and an axes.
    step = 1
    ax_cbf_extra_robot.axis('on')
    for i in range(1, len(cbf_extra_robot_col)):
        if i > 0:
            ax_cbf_extra_robot.plot(df_cbf_extra_robot[cbf_extra_robot_col[0]].iloc[starting_point:-1], df_cbf_extra_robot[cbf_extra_robot_col[i]].iloc[starting_point:-1], label="Robot"+str(step))  # Plot some data on the axes.
            step += 1

    ax_cbf_extra_robot.set_xlabel('time')  # Add an x-label to the axes.
    ax_cbf_extra_robot.set_ylabel('h_extra_robot')  # Add a y-label to the axes.
    ax_cbf_extra_robot.set_title("CBF functions for extra_robot")  # Add a title to the axes.
    ax_cbf_extra_robot.legend()  # Add a legend.
    ax_cbf_extra_robot.axhline(y=0, color='k', lw=1)

plt.show()


print("Showing animation...")

# Start figure and axes with limits
fig = plt.figure()
ax = plt.axes(xlim=(-winx, winx), ylim=(-winy, winy))

time_txt = ax.text(0.475, 0.975,'',horizontalalignment='left',verticalalignment='top', transform=ax.transAxes)

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
if wedge:
    wedge1 = plt.Line2D((-x_max, x_max), (y_max, 0), lw=1, color='r', alpha=0.7)
    wedge2 = plt.Line2D((-x_max, x_max), (-y_max, 0), lw=1, color='r', alpha=0.7)
    plt.gca().add_line(wedge1)
    plt.gca().add_line(wedge2)

shapes = []
for i in range(number_robots):
    shapes.append(plt.Circle((p[2*i,0], p[2*i+1,0]), r_robot, fc='b'))

for i in range(len(edges)):
    aux_i = edges[i][0]-1
    aux_j = edges[i][1]-1
    shapes.append(plt.Line2D((p[2*aux_i,0], p[2*aux_j,0]), (p[2*aux_i+1,0], p[2*aux_j+1,0]), lw=0.5, color='b', alpha=0.3))

if extra_robot:
    shapes.append(plt.Circle((huil_p[0,0], huil_p[1,0]), r_robot, fc='g'))

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

    if extra_robot:
        shapes[-1].center = (huil_p[0,0], huil_p[1,0])
        ax.add_patch(shapes[-1])

    time_txt.set_text('T=0.0 s')

    return shapes + [time_txt,]

def animate(frame):

    for i in range(number_robots):
        shapes[i].center = (p[2*i,frame], p[2*i+1,frame])

    for i in range(len(edges)):
        aux_i = edges[i][0]-1
        aux_j = edges[i][1]-1
        shapes[number_robots+i].set_xdata((p[2*aux_i,frame], p[2*aux_j,frame]))
        shapes[number_robots+i].set_ydata((p[2*aux_i+1,frame], p[2*aux_j+1,frame]))

    if extra_robot:
        shapes[-1].center = (huil_p[0,frame], huil_p[1,frame])

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