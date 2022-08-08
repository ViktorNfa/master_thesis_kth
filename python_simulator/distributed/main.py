#=====================================
#          Python simulator
#          mobile 2D robots
#     Victor Nan Fernandez-Ayala 
#           (vnfa@kth.se)
#=====================================

import numpy as np
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
freq = 400

# Frequency of update of the control solver
freq_sol = 50

# Maximum time of the simulation (in seconds)
max_T = 30

# Ideal formation positions
formation_positions = [[0, 2], [0, 0], [0, -2], [2, 2], [2, -2]]
#formation_positions = [[0, 10], [0, 8], [0, 6], [0, 4], [0, 2], [0, 0], [0, -2], [0, -4], [0, -6], [0, -8], [0, -10], 
#                       [10, 10], [8, 8], [6, 6], [4, 4], [2, 2], [2, -2], [4, -4], [6, -6], [8, -8], [10, -10]]

# Get the number of robots
number_robots = len(formation_positions)

# List of neighbours for each robot
#neighbours = [[2], [1, 3, 4, 5], [2], [2], [2]]
#neighbours = [[2, 4], [1, 3, 4, 5], [2, 5], [1, 2], [2, 3]]
#neighbours = [[2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7, 16, 17], [6, 8], [7, 9], [8, 10], [9, 11], [10], 
#               [13], [12, 14], [13, 15], [14, 16], [6, 15], [6, 18], [17, 19], [18, 20], [19, 21], [20]]
neighbours = [[i+1 for i in range(number_robots) if i != j] for j in range(number_robots)]

# CBF Communication maintenance or obstacle avoidance activation 
# (1 is activated/0 is deactivated)
cm = 0
oa = 1

# Safe distance for communication maintenance and obstacle avoidance
d_cm = 3
d_oa = 1.1

# Linear alpha function with parameter
alpha = 1

# Exponential parameter
p = 1

# Adaptative law parameter
k0 = 1

# Maximum value of control input
u_max = 10

# Variable to determine if HuIL is active or not 
# (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1
human_robot = number_robots

# HuIL parameters
v_huil = 2
division = 6

# Parameter to decide if Extra robot is shown or not
extra_robot = False


## Pre-calculations needed for controller and simulation

# Time size
max_time_size = max_T*freq

# Get the number of neighbours for each robot
number_neighbours = []
# Create edge list
edges = []
# Create Laplacian matrix for the graph
L_G = np.zeros((number_robots,number_robots))
# Setup controller output init output
controller = np.zeros((number_robots*dim,max_time_size))
nom_controller = np.zeros((number_robots*dim,max_time_size))
huil_controller = np.zeros((dim,max_time_size))
cbf_extra_robot = np.zeros((number_robots*dim,max_time_size))
for i in range(number_robots):
    number_neighbours.append(len(neighbours[i]))
    L_G[i, i] = number_neighbours[i]
    for j in neighbours[i]:
        L_G[i, j-1] = -1
        if (i+1,j) not in edges and (j,i+1) not in edges:
            edges.append((i+1,j))


# Create edge list for the name of columns
edges_col = []
# Setup cbf functions init output
cbf_cm = np.zeros((len(edges),max_time_size))
cbf_oa = np.zeros((len(edges),max_time_size))
for i in range(len(edges)):
    edges_col.append("Edge"+str(edges[i]))

# Modify ideal formation positions to one column vector
x_d = np.reshape(formation_positions,number_robots*dim)

# Update parameter for multi-freq update
update_par = freq/freq_sol


## Simulation and visualization loop

# Initialize position matrix
x = np.zeros((number_robots*dim,max_time_size))

# Initialize position for extra robot
huil_x = np.zeros((dim,max_time_size))

# Randomize initial position
#x[:,0] = 1*(2*x_max*np.random.rand(number_robots*dim)-x_max)
#x[:,0] = [-(x_max-1), (y_max-1), 0, 0, -(x_max-1), -(y_max-1), (x_max-1), (y_max-1), (x_max-1), -(y_max-1)]
#x[:,0] = [-(x_max-5), (y_max-5), 0, 0, -(x_max-5), -(y_max-5), (x_max-5), (y_max-5), (x_max-5), -(y_max-5)]
x[:,0] = x_max*np.random.rand(number_robots*dim)-x_max/2
#x[:,0] = np.array([0, 2, 0, 0, 0, -2, 2, 2, 2, -2])
#x[:,0] = 2*np.random.rand(number_robots*dim)-2/2
#x[:,0] = 0.7*np.ones(number_robots*dim)+0.5*np.random.rand(number_robots*dim)
#x[:,0] = 0.7*np.ones(number_robots*dim)

# Random initial position for extra robot
huil_x[:,0] = np.array([-(x_max-5), (y_max-1)])

# Initialize slack variable
y = np.zeros((number_robots,max_time_size))

# Define objective function for minimization solver
def objective_function(u_sol, u_n_sol):
    return np.linalg.norm(u_sol - u_n_sol)**2

# Start simulation loop
print("Computing evolution of the system...")
for t in tqdm(range(max_time_size-1)):
    secs = t/freq

    if t % update_par == 0:
        # Compute nominal controller - Distributed
        u_nom = np.zeros((dim*number_robots, 1))
        for i in range(number_robots):
            u_nom[2*i:2*i+2] = np.expand_dims(formationController(i, number_neighbours[i], neighbours[i], x[:,t], x_d), axis=1)

        #u_nom = formationControllerCentralized(L_G, x[:,t], x_d)
        u_nom = np.squeeze(u_nom, axis=1)

        u_n = huilController(u_nom, huil, human_robot, t, max_time_size, v_huil, division)

        # Compute CBF constrained controller - Distributed
        u = np.zeros((dim*number_robots,1))
        c = np.zeros((number_robots, 1))
        a = np.zeros((dim*number_robots, 1))
        b = np.zeros((number_robots, 1))
        for i in range(number_robots):

            # Collective constraints
            for e in range(len(edges)):
                aux_i = edges[e][0]-1
                aux_j = edges[e][1]-1
                x_i = np.array([x[2*aux_i,t],x[2*aux_i+1,t]])
                x_j = np.array([x[2*aux_j,t],x[2*aux_j+1,t]])

                if i == aux_i:
                    # CM
                    a[2*i:2*i+2] += -p*np.exp(-p*cbf_h(x_i, x_j, d_cm, 1))*cbf_gradh(x_i, x_j, 1)
                    b[i] += -alpha/2*(1/len(edges)-np.exp(-p*cbf_h(x_i, x_j, d_cm, 1)))
                    # OA
                    a[2*i:2*i+2] += -p*np.exp(-p*cbf_h(x_i, x_j, d_oa, -1))*cbf_gradh(x_i, x_j, -1)
                    b[i] += -alpha/2*(1/len(edges)-np.exp(-p*cbf_h(x_i, x_j, d_oa, -1)))
                elif i == aux_j:
                    # CM
                    a[2*i:2*i+2] += -p*np.exp(-p*cbf_h(x_i, x_j, d_cm, 1))*cbf_gradh(x_j, x_i, 1)
                    b[i] += -alpha/2*(1/len(edges)-np.exp(-p*cbf_h(x_i, x_j, d_cm, 1)))
                    # OA
                    a[2*i:2*i+2] += -p*np.exp(-p*cbf_h(x_i, x_j, d_oa, -1))*cbf_gradh(x_j, x_i, -1)
                    b[i] += -alpha/2*(1/len(edges)-np.exp(-p*cbf_h(x_i, x_j, d_oa, -1)))
                else:
                    # CM
                    a[2*i:2*i+2] += np.zeros((dim, 1))
                    b[i] += 0
                    # OA
                    a[2*i:2*i+2] += np.zeros((dim, 1))
                    b[i] += 0

            # Individual constraints
            
            
            u[2*i:2*i+2] = np.expand_dims(u_n[2*i:2*i+2], axis=1)
            if a[2*i] != 0 and a[2*i+1] != 0:
                c[i] = (np.dot(L_G[i,:],y[:,t]) + 
                    np.dot(np.transpose(a[2*i:2*i+2]),u_n[2*i:2*i+2])+b[i])/np.dot(np.transpose(a[2*i:2*i+2]),a[2*i:2*i+2])
                u[2*i:2*i+2] -=  np.maximum(0,c[i])*a[2*i:2*i+2]

        u = np.squeeze(u, axis=1)

    # Update the system using dynamics
    xdot = systemDynamics(x[:,t], u, u_max, -u_max)
    x[:,t+1] = xdot*(1/freq) + x[:,t]

    # Update slack variable
    for i in range(number_robots):
        if a[2*i] == 0 and a[2*i+1] == 0:
            y[i,t+1] = 0
        else:
            #y[i,t+1] = y[i,t]
            y[i,t+1] = y[i,t] - np.transpose(k0*sigmoid(np.dot(L_G[i,:],c), 10))*(1/freq)

    # Save CBF functions
    for e in range(len(edges)):
        aux_i = edges[e][0]-1
        aux_j = edges[e][1]-1
        x_i = np.array([x[2*aux_i,t],x[2*aux_i+1,t]])
        x_j = np.array([x[2*aux_j,t],x[2*aux_j+1,t]])
        cbf_cm[e,t+1] = cbf_h(x_i, x_j, d_cm, 1)
        cbf_oa[e,t+1] = cbf_h(x_i, x_j, d_oa, -1)
    
    # Save Final controller
    controller[:,t+1] = u

    # Save Nominal controller
    nom_controller[:,t+1] = u_nom

    # Save HuIL controller
    huil_controller[:,t+1] = np.array([u_n[2*human_robot-2], u_n[2*human_robot-1]])


## Visualize conditions/plots & trajectories

print("Showing functions evolution...")

# Plot y-variables
fig_y, ax_y = plt.subplots()  # Create a figure and an axes.
for i in range(number_robots):
    ax_y.plot(1/freq*np.arange(max_time_size), y[i,:], label="Robot"+str(i+1))  # Plot some data on the axes.
ax_y.set_xlabel('time')  # Add an x-label to the axes.
ax_y.set_ylabel('y')  # Add a y-label to the axes.
ax_y.set_title("y-variables")  # Add a title to the axes.
ax_y.legend()  # Add a legend.
ax_y.axhline(y=0, color='k', lw=1)

# Plot the CBF comunication maintenance
fig_cbf_cm, ax_cbf_cm = plt.subplots()  # Create a figure and an axes.
for i in range(len(edges)):
    ax_cbf_cm.plot(1/freq*np.arange(max_time_size), cbf_cm[i,:], label=edges_col[i])  # Plot some data on the axes.
ax_cbf_cm.set_xlabel('time')  # Add an x-label to the axes.
ax_cbf_cm.set_ylabel('h_cm')  # Add a y-label to the axes.
ax_cbf_cm.set_title("CBF functions for comunication maintenance")  # Add a title to the axes.
ax_cbf_cm.legend()  # Add a legend.
ax_cbf_cm.axhline(y=0, color='k', lw=1)

# Plot the CBF obstacle avoidance
fig_cbf_oa, ax_cbf_oa = plt.subplots()  # Create a figure and an axes.
for i in range(len(edges)):
    ax_cbf_oa.plot(1/freq*np.arange(max_time_size), cbf_oa[i,:], label=edges_col[i])  # Plot some data on the axes.
ax_cbf_oa.set_xlabel('time')  # Add an x-label to the axes.
ax_cbf_oa.set_ylabel('h_oa')  # Add a y-label to the axes.
ax_cbf_oa.set_title("CBF functions for obstacle avoidance")  # Add a title to the axes.
ax_cbf_oa.legend()  # Add a legend.
ax_cbf_oa.axhline(y=0, color='k', lw=1)

# Plot the normed difference between nominal and final controller
fig_norm, ax_norm = plt.subplots()  # Create a figure and an axes.
ax_norm.axis('on')
for i in range(number_robots):
    diff_x = controller[2*i,:] - nom_controller[2*i,:]
    diff_y = controller[2*i+1,:] - nom_controller[2*i+1,:]
    diff = np.array([diff_x, diff_y])
    normed_difference = np.sqrt(np.square(diff).sum(axis=0))
    ax_norm.plot(1/freq*np.arange(max_time_size), normed_difference, label="Robot"+str(i+1))  # Plot some data on the axes.

diff_x = controller[2*human_robot-2,:] - huil_controller[0,:]
diff_y = controller[2*human_robot-1,:] - huil_controller[1,:]
diff = np.array([diff_x, diff_y])
normed_difference = np.sqrt(np.square(diff).sum(axis=0))
ax_norm.plot(1/freq*np.arange(max_time_size), normed_difference, label="HuILDiff"+str(human_robot))  # Plot some data on the axes.

huil = np.array([huil_controller[0,:], huil_controller[1,:]])
normed_huil = np.sqrt(np.square(huil).sum(axis=0))
#ax_norm.plot(1/freq*np.arange(max_time_size), normed_huil, label="HuIL"+str(human_robot))  # Plot some data on the axes.

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

# Add initial points
initials = []
for i in range(number_robots):
    initials.append(plt.Circle((x[2*i,0], x[2*i+1,0]), r_robot/2, fc='k', alpha=0.3))
    plt.gca().add_patch(initials[i])
for i in range(len(edges)):
    aux_i = edges[i][0]-1
    aux_j = edges[i][1]-1
    initials.append(plt.Line2D((x[2*aux_i,0], x[2*aux_j,0]), (x[2*aux_i+1,0], x[2*aux_j+1,0]), lw=0.5, color='k', alpha=0.1))
    plt.gca().add_line(initials[number_robots+i])

# Add the limits of the arena
arena_limit1 = plt.Line2D((-x_max, x_max), (y_max, y_max), lw=2.5, color='r')
arena_limit2 = plt.Line2D((-x_max, x_max), (-y_max, -y_max), lw=2.5, color='r')
arena_limit3 = plt.Line2D((x_max, x_max), (-y_max, y_max), lw=2.5, color='r')
arena_limit4 = plt.Line2D((-x_max, -x_max), (-y_max, y_max), lw=2.5, color='r')
plt.gca().add_line(arena_limit1)
plt.gca().add_line(arena_limit2)
plt.gca().add_line(arena_limit3)
plt.gca().add_line(arena_limit4)

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