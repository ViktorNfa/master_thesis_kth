#!/usr/bin/env python
#=====================================
#       Module for plotting the
#      data saved by the logger
#         in a pickle file
#=====================================

import matplotlib.pyplot as plt 
import pandas as pd
import numpy as np
import time

#plt.style.use('fivethirtyeight')

arrow = True

#Names of the saved logs
cbf_cm_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_cm_log.csv'
cbf_oa_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_oa_log.csv'
cbf_arena_top_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_arena_top_log.csv'
cbf_arena_right_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_arena_right_log.csv'
cbf_arena_bottom_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_arena_bottom_log.csv'
cbf_arena_left_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_arena_left_log.csv'
cbf_obstacle_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_obstacle_log.csv'
cbf_extra_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/cbf_extra_log.csv'
controller_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/controller_log.csv'
nom_controller_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/nom_controller_log.csv'
huil_controller_filename = '/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/auxiliary_code/huil_controller_log.csv'

#---------------
# Read the data
#---------------
#For the CBF functions
df_cbf_cm = pd.read_csv(cbf_cm_filename)
df_cbf_oa = pd.read_csv(cbf_oa_filename)
df_cbf_arena_top = pd.read_csv(cbf_arena_top_filename)
df_cbf_arena_right = pd.read_csv(cbf_arena_right_filename)
df_cbf_arena_bottom = pd.read_csv(cbf_arena_bottom_filename)
df_cbf_arena_left = pd.read_csv(cbf_arena_left_filename)
df_cbf_obstacle = pd.read_csv(cbf_obstacle_filename)
df_cbf_extra = pd.read_csv(cbf_extra_filename)

#For the final, nominal and HuIL controller
df_controller = pd.read_csv(controller_filename)
df_nom_controller = pd.read_csv(nom_controller_filename)
df_huil_controller = pd.read_csv(huil_controller_filename)

#---------------
# Plot the data
#---------------
#Plot the CBF comunication maintenance
cbf_cm_col = df_cbf_cm.columns.values
starting_point = df_cbf_cm[cbf_cm_col[1]].ne(0).idxmax()
fig_cbf_cm, ax_cbf_cm = plt.subplots()  # Create a figure and an axes.
for i in range(len(cbf_cm_col)):
    if i > 0:
        ax_cbf_cm.plot(df_cbf_cm[cbf_cm_col[0]].iloc[starting_point:-1], df_cbf_cm[cbf_cm_col[i]].iloc[starting_point:-1], label=cbf_cm_col[i])  # Plot some data on the axes.

ax_cbf_cm.set_xlabel('time')  # Add an x-label to the axes.
ax_cbf_cm.set_ylabel('h_cm')  # Add a y-label to the axes.
ax_cbf_cm.set_title("CBF functions for comunication maintenance")  # Add a title to the axes.
ax_cbf_cm.legend()  # Add a legend.
ax_cbf_cm.axhline(y=0, color='k', lw=1)

#Plot the CBF obstacle avoidance
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

# Plot the CBF arena walls
cbf_arena_col = df_cbf_arena_top.columns.values
fig_cbf_arena, ax_cbf_arena = plt.subplots()  # Create a figure and an axes.
for i in range(1, len(cbf_arena_col)):
    ax_cbf_arena.plot(df_cbf_arena_top[cbf_arena_col[0]].iloc[starting_point:-1], df_cbf_arena_top[cbf_arena_col[i]].iloc[starting_point:-1], label="Robot_Top"+str(i+1))  # Plot some data on the axes.
    ax_cbf_arena.plot(df_cbf_arena_top[cbf_arena_col[0]].iloc[starting_point:-1], df_cbf_arena_right[cbf_arena_col[i]].iloc[starting_point:-1], label="Robot_Right"+str(i+1))  # Plot some data on the axes.
    ax_cbf_arena.plot(df_cbf_arena_top[cbf_arena_col[0]].iloc[starting_point:-1], df_cbf_arena_bottom[cbf_arena_col[i]].iloc[starting_point:-1], label="Robot_Bottom"+str(i+1))  # Plot some data on the axes.
    ax_cbf_arena.plot(df_cbf_arena_top[cbf_arena_col[0]].iloc[starting_point:-1], df_cbf_arena_left[cbf_arena_col[i]].iloc[starting_point:-1], label="Robot_Left"+str(i+1))  # Plot some data on the axes.
ax_cbf_arena.set_xlabel('time')  # Add an x-label to the axes.
ax_cbf_arena.set_ylabel('h_arena')  # Add a y-label to the axes.
ax_cbf_arena.set_title("CBF functions for arena walls")  # Add a title to the axes.
ax_cbf_arena.legend()  # Add a legend.
ax_cbf_arena.axhline(y=0, color='k', lw=1)

# Plot the CBF obstacle
cbf_obstacle_col = df_cbf_obstacle.columns.values
fig_cbf_obstacle, ax_cbf_obstacle = plt.subplots()  # Create a figure and an axes.
for i in range(1, len(cbf_obstacle_col)):
    ax_cbf_obstacle.plot(df_cbf_obstacle[cbf_obstacle_col[0]].iloc[starting_point:-1], df_cbf_obstacle[cbf_obstacle_col[i]].iloc[starting_point:-1], label="Robot"+str(i+1))  # Plot some data on the axes.
ax_cbf_obstacle.set_xlabel('time')  # Add an x-label to the axes.
ax_cbf_obstacle.set_ylabel('h_center_obstacle')  # Add a y-label to the axes.
ax_cbf_obstacle.set_title("CBF functions for center obstacle")  # Add a title to the axes.
ax_cbf_obstacle.legend()  # Add a legend.
ax_cbf_obstacle.axhline(y=0, color='k', lw=1)

# Plot the CBF extra robot
cbf_extra_col = df_cbf_extra.columns.values
fig_cbf_extra, ax_cbf_extra = plt.subplots()  # Create a figure and an axes.
for i in range(1, len(cbf_obstacle_col)):
    ax_cbf_extra.plot(df_cbf_extra[cbf_extra_col[0]].iloc[starting_point:-1], df_cbf_extra[cbf_extra_col[i]].iloc[starting_point:-1], label="Robot"+str(i+1))  # Plot some data on the axes.
ax_cbf_extra.set_xlabel('time')  # Add an x-label to the axes.
ax_cbf_extra.set_ylabel('h_extra')  # Add a y-label to the axes.
ax_cbf_extra.set_title("CBF functions for extra human")  # Add a title to the axes.
ax_cbf_extra.legend()  # Add a legend.
ax_cbf_extra.axhline(y=0, color='k', lw=1)

#Plot the normed difference between nominal and final controller
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

ax_norm.set_xlabel('time')  # Add an x-label to the axes.
ax_norm.set_ylabel('|u - u_nom|')  # Add a y-label to the axes.
ax_norm.set_title("Normed difference between u and nominal u")  # Add a title to the axes.
ax_norm.legend()  # Add a legend.
ax_norm.axhline(y=0, color='k', lw=1)

plt.show()

#For robot 4
show_action = False
if show_action:
    plt.ion()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    img = plt.imread("/home/victor/catkin_ws/src/cbf-enabled-hil-control-mas/first_task_formation_cbf/src/nexus.png")
    ax.imshow(img)
    ax.set_title("HuIL controller and CBF-QP controller output")
    ax.axis('off')

    arrow1 = plt.arrow(x=579, y=373, dx=200*0, dy=100*0, width=10, facecolor='red', edgecolor='none')
    arrow2 = plt.arrow(x=579, y=373, dx=200*0, dy=100*0, width=10, facecolor='blue', edgecolor='none')

    plt.legend([arrow1, arrow2,], ['HuIL', 'CBF-QP',])

    ax.plot(579, 373, '-ko')
    for i in range(len(df_controller[controller_col[-1]].iloc[starting_point:-1])):
        arrow1.remove()
        arrow2.remove()
        ux = df_controller[controller_col[-2]].iloc[starting_point+i]
        uy = df_controller[controller_col[-1]].iloc[starting_point+i]
        uhuilx = df_huil_controller[controller_col[-2]].iloc[starting_point+i]
        uhuily = df_huil_controller[controller_col[-1]].iloc[starting_point+i]
        arrow1 = plt.arrow(x=579, y=373, dx=-300*uhuily, dy=-300*uhuilx, width=10, facecolor='red', edgecolor='none')
        arrow2 = plt.arrow(x=579, y=373, dx=-300*uy, dy=-300*ux, width=10, facecolor='blue', edgecolor='none')
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.02)

