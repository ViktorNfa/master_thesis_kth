#!/usr/bin/env python
#=====================================
#       Module for plotting the
#      data saved by the logger
#         in a pickle file
#=====================================

import matplotlib.pyplot as plt 
import pandas as pd
import numpy as np

plt.style.use('fivethirtyeight')

arrow = True

#Names of the saved logs
cbf_cm_filename = '/home/viktornfa/catkin_ws/src/master_thesis_victor/auxiliary_code/cbf_cm_log.csv'
cbf_oa_filename = '/home/viktornfa/catkin_ws/src/master_thesis_victor/auxiliary_code/cbf_oa_log.csv'
controller_filename = '/home/viktornfa/catkin_ws/src/master_thesis_victor/auxiliary_code/controller_log.csv'
nom_controller_filename = '/home/viktornfa/catkin_ws/src/master_thesis_victor/auxiliary_code/nom_controller_log.csv'

#---------------
# Read the data
#---------------
#For the CBF functions
df_cbf_cm = pd.read_csv(cbf_cm_filename)
df_cbf_oa = pd.read_csv(cbf_oa_filename)

#For the final controller
df_controller = pd.read_csv(controller_filename)
df_nom_controller = pd.read_csv(nom_controller_filename)

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

#For robot 5
show_action = False
if show_action:
    plt.ion()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    img = plt.imread("/home/viktornfa/catkin_ws/src/master_thesis_victor/first_task_formation_cbf/src/nexus.png")
    ax.imshow(img)
    ax.set_title("HuIL controller and CBF-QP controller output")
    ax.axis('off')

    arrow1 = plt.arrow(x=579, y=373, dx=200*0, dy=100*0, width=10, facecolor='red', edgecolor='none')
    arrow2 = plt.arrow(x=579, y=373, dx=200*0, dy=100*0, width=10, facecolor='blue', edgecolor='none')

    plt.legend([arrow1, arrow2,], ['HuIL', 'CBF-QP',])

    ax.plot(579, 373, '-ko')
    for i in range(len(df_controller[controller_col[9]].iloc[starting_point:-1])):
        arrow1.remove()
        arrow2.remove()
        ux = df_controller[controller_col[-2]].iloc[starting_point+i]
        uy = df_controller[controller_col[-1]].iloc[starting_point+i]
        unx = df_nom_controller[controller_col[-2]].iloc[starting_point+i]
        uny = df_nom_controller[controller_col[-1]].iloc[starting_point+i]
        arrow1 = plt.arrow(x=579, y=373, dx=100*unx, dy=100*uny, width=10, facecolor='red', edgecolor='none')
        arrow2 = plt.arrow(x=579, y=373, dx=100*uy, dy=100*uy, width=10, facecolor='blue', edgecolor='none')
        fig.canvas.draw()
        fig.canvas.flush_events()
        print(i)

