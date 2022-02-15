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