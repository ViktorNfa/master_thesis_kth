import numpy as np

## Auxiliary functions

def formationController(L_G, p, p_d):
    # Create extended laplacian
    I = np.identity(2)
    L_ext = np.kron(L_G,I)

    # Compute formation controller
    u = -np.dot(L_ext,p-p_d)
    return u

def huilController(u_nom, huil, human_robot, i, max_time_size):
    # HuIL parameters
    v_huil = 3
    division = 6

    # Leave some time at the start and the end to allow the robots to form
    max_time_size = max_time_size - 2*max_time_size/division
    i = i - max_time_size/division

    # Simulated HuIL input to move in  a rectangle manner
    if huil > 0:
        if i < max_time_size/4 and i > 0:
            u_huil_x = 0
            u_huil_y = -v_huil
        elif i > max_time_size/4 and i < max_time_size/2:
            u_huil_x = v_huil
            u_huil_y = 0
        elif i > max_time_size/2 and i < 3*max_time_size/4:
            u_huil_x = 0
            u_huil_y = v_huil
        elif i > 3*max_time_size/4 and i <= max_time_size:
            u_huil_x = -v_huil
            u_huil_y = 0
        else:
            u_huil_x = 0
            u_huil_y = 0
    
    u_nom[2*human_robot-2] += u_huil_x
    u_nom[2*human_robot-1] += u_huil_y

    return u_nom

def cbfController(p, u, cm, oa, d_cm, d_oa, edges):
    return u

def systemDynamics(p, u):
    # System dynamics parameters
    f = np.zeros(len(p))
    g = np.identity(len(u))

    # Update state vector derivative
    xdot = f+np.dot(g,u)
    return xdot