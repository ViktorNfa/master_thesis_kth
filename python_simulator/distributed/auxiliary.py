import numpy as np


## Auxiliary functions

def formationControllerCentralized(L_G, p, p_d):
    # Create extended laplacian
    I = np.identity(2)
    L_ext = np.kron(L_G,I)

    # Compute formation controller
    u = -np.dot(L_ext,p-p_d)
    return u

def formationController(i, number_neighbours, neighbours, p, p_d):
    u = np.zeros(2)
    for j in range(number_neighbours):
        aux_j = neighbours[j]
        u += p[2*aux_j-2:2*aux_j] - p[2*i:2*i+2] + p_d[2*i:2*i+2] - p_d[2*aux_j-2:2*aux_j]
    
    return u

def coverageController(p, p_d, gain):
    u = gain*(np.squeeze(p_d, axis=1) - p)
    
    return u

def huilController(u_nom, huil, human_robot, i, max_time_size, v_huil, division):
    # Leave some time at the start and the end to allow the robots to form
    max_time = max_time_size*(1-2/division)
    i = i - max_time_size/division

    u_huil_x = 0
    u_huil_y = 0
    # Simulated HuIL input to move in  a rectangle manner
    if huil > 0:
        if i < max_time/4 and i > 0:
            u_huil_x = 0
            u_huil_y = -v_huil
        elif i > max_time/4 and i < max_time/2:
            u_huil_x = v_huil
            u_huil_y = 0
        elif i > max_time/2 and i < 3*max_time/4:
            u_huil_x = 0
            u_huil_y = v_huil
        elif i > 3*max_time/4 and i <= max_time:
            u_huil_x = -v_huil
            u_huil_y = 0
        else:
            u_huil_x = 0
            u_huil_y = 0
    
    u_nom[2*human_robot-2] += u_huil_x
    u_nom[2*human_robot-1] += u_huil_y

    return u_nom

def extraRobotDynamics(i, max_time_size, v_huil, division):
    # Leave some time at the start and the end to allow the robots to form
    max_time = max_time_size*(1-2/division)
    i = i - max_time_size/division

    u_huil_x = 0
    u_huil_y = 0
    # Simulated HuIL input to move in  a rectangle manner
    if i < max_time/4 and i > 0:
        u_huil_x = 0
        u_huil_y = -3*v_huil/5
    elif i > max_time/4 and i < max_time/2:
        u_huil_x = v_huil
        u_huil_y = 0
    elif i > max_time/2 and i < 3*max_time/4:
        u_huil_x = 0
        u_huil_y = v_huil/5
    elif i > 3*max_time/4 and i <= max_time:
        u_huil_x = -v_huil
        u_huil_y = 0
    else:
        u_huil_x = 0
        u_huil_y = 0

    return np.array([u_huil_x, u_huil_y])

def cbf_h(p_i, p_j, safe_distance, dir):
    # Dir 1 corresponds to CM and -1 to OA
    return dir*(safe_distance**2 - np.linalg.norm(p_i - p_j)**2)

def cbf_gradh(p_i, p_j, dir):
    # Dir 1 corresponds to CM and -1 to OA
    return dir*(-2*np.array([[p_i[0]-p_j[0]], [p_i[1]-p_j[1]]]))

def cbf_walls(p, wall):
    return wall[2]*(wall[0]-p[wall[1]])

def systemDynamics(p, u, u_max, u_min):
    # System dynamics parameters
    f = np.zeros(len(p))
    g = np.identity(len(u))

    # Bound the control input
    for i in range(len(u)):
        u[i] = max(u_min, min(u_max, u[i]))

    # Update state vector derivative
    xdot = f+np.dot(g,u)
    return xdot

def sign_filter(x, a):
    if x >= a:
        sol = 1
    elif x <= a:
        sol = -1
    else:
        sol = 1/a*x

    return sol