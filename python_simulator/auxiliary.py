#!/usr/bin/env python
import numpy as np
from scipy.optimize import minimize, LinearConstraint


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

    u_huil_x = 0
    u_huil_y = 0
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

def cbf_h(p_i, p_j, safe_distance, dir):
    # Dir 1 corresponds to CM and -1 to OA
    return dir*(safe_distance**2 - np.linalg.norm(p_i - p_j)**2)

def cbf_gradh(p_i, p_j, dir):
    # Dir 1 corresponds to CM and -1 to OA
    return dir*(-2*np.array([[p_i[0]-p_j[0]], [p_i[1]-p_j[1]]]))

def cbfController(p, u_n, cm, oa, d_cm, d_oa, number_robots, edges, n, alpha):
    #Create CBF constraint matrices
    A_cm = np.zeros((len(edges), number_robots*n))
    b_cm = np.zeros((len(edges)))
    A_oa = np.zeros((len(edges), number_robots*n))
    b_oa = np.zeros((len(edges)))
    for i in range(len(edges)):
        aux_i = edges[i][0]-1
        aux_j = edges[i][1]-1
        p_i = np.array([p[2*aux_i],p[2*aux_i+1]])
        p_j = np.array([p[2*aux_j],p[2*aux_j+1]])

        b_cm[i] = alpha*cbf_h(p_i, p_j, d_cm, 1)
        b_oa[i] = alpha*cbf_h(p_i, p_j, d_oa, -1)

        grad_h_value_cm = np.transpose(cbf_gradh(p_i, p_j, 1))
        grad_h_value_oa = np.transpose(cbf_gradh(p_i, p_j, -1))

        A_cm[i, 2*aux_i:2*aux_i+2] = grad_h_value_cm
        A_cm[i, 2*aux_j:2*aux_j+2] = -grad_h_value_cm
        A_oa[i, 2*aux_i:2*aux_i+2] = grad_h_value_oa
        A_oa[i, 2*aux_j:2*aux_j+2] = -grad_h_value_oa

    #----------------------------
    # Solve minimization problem
    #----------------------------
    #Define linear constraints
    constraint_cm = LinearConstraint(A_cm*cm, lb=-b_cm*cm, ub=np.inf)
    constraint_oa = LinearConstraint(A_oa*oa, lb=-b_oa*oa, ub=np.inf)
    
    #Define objective function
    def objective_function(u, u_n):
        return np.linalg.norm(u - u_n)**2
    
    #Construct the problem
    u = minimize(
        objective_function,
        x0=u_n,
        args=(u_n,),
        constraints=[constraint_cm, constraint_oa],
    )

    return u.x

def systemDynamics(p, u):
    # System dynamics parameters
    f = np.zeros(len(p))
    g = np.identity(len(u))

    # Update state vector derivative
    xdot = f+np.dot(g,u)
    return xdot