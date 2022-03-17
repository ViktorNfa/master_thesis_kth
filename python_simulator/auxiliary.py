#!/usr/bin/env python

# To force int division to floats (for Python 2.7)
from __future__ import division

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

    return np.array([u_huil_x, u_huil_y])

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

def cbfControllerWArena(p, u_n, cm, oa, d_cm, d_oa, number_robots, edges, n, alpha, A_arena, b_arena, x_max, x_min, y_max, y_min):
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

    #Calculate CBF for arena safety
    for i in range(number_robots):
        b_arena[4*i] = alpha*(x_max - p[2*i])
        b_arena[4*i+1] = alpha*(p[2*i] - x_min)
        b_arena[4*i+2] = alpha*(y_max - p[2*i+1])
        b_arena[4*i+3] = alpha*(p[2*i+1] - y_min)

    #----------------------------
    # Solve minimization problem
    #----------------------------
    #Define linear constraints
    constraint_cm = LinearConstraint(A_cm*cm, lb=-b_cm*cm, ub=np.inf)
    constraint_oa = LinearConstraint(A_oa*oa, lb=-b_oa*oa, ub=np.inf)
    constraint_arena = LinearConstraint(A_arena, lb=-b_arena, ub=np.inf)
    
    #Define objective function
    def objective_function(u, u_n):
        return np.linalg.norm(u - u_n)**2
    
    #Construct the problem
    u = minimize(
        objective_function,
        x0=u_n,
        args=(u_n,),
        constraints=[constraint_cm, constraint_oa, constraint_arena],
    )

    return u.x

def cbfControllerWArenaWedge(p, u_n, cm, oa, d_cm, d_oa, number_robots, edges, n, alpha, A_arena, b_arena, x_max, x_min, y_max, y_min, A_wedge, b_wedge):
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

    #Calculate CBF for arena safety
    for i in range(number_robots):
        b_arena[4*i] = alpha*(x_max - p[2*i])
        b_arena[4*i+1] = alpha*(p[2*i] - x_min)
        b_arena[4*i+2] = alpha*(y_max - p[2*i+1])
        b_arena[4*i+3] = alpha*(p[2*i+1] - y_min)

    #Calculate CBF for wedge
    for i in range(number_robots):
        b_wedge[2*i] = alpha*(-y_max/(2*x_max)*p[2*i] + y_max/2 - p[2*i+1])
        b_wedge[2*i+1] = alpha*(p[2*i+1] - y_max/(2*x_max)*p[2*i] + y_max/2)

    #----------------------------
    # Solve minimization problem
    #----------------------------
    #Define linear constraints
    constraint_cm = LinearConstraint(A_cm*cm, lb=-b_cm*cm, ub=np.inf)
    constraint_oa = LinearConstraint(A_oa*oa, lb=-b_oa*oa, ub=np.inf)
    constraint_arena = LinearConstraint(A_arena, lb=-b_arena, ub=np.inf)
    constraint_wedge = LinearConstraint(A_wedge, lb=-b_wedge, ub=np.inf)
    
    #Define objective function
    def objective_function(u, u_n):
        return np.linalg.norm(u - u_n)**2
    
    #Construct the problem
    u = minimize(
        objective_function,
        x0=u_n,
        args=(u_n,),
        constraints=[constraint_cm, constraint_oa, constraint_arena, constraint_wedge],
    )

    return u.x

def cbfControllerWArenaExtra(p, u_n, cm, oa, d_cm, d_oa, number_robots, edges, n, alpha, A_arena, b_arena, x_max, x_min, y_max, y_min, A_extra, b_extra, d_extra, huil_p, vxe, vye):
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

    #Calculate CBF for arena safety
    for i in range(number_robots):
        b_arena[4*i] = alpha*(x_max - p[2*i])
        b_arena[4*i+1] = alpha*(p[2*i] - x_min)
        b_arena[4*i+2] = alpha*(y_max - p[2*i+1])
        b_arena[4*i+3] = alpha*(p[2*i+1] - y_min)

        A_extra[i, 2*i:2*i+2] = 2*np.array([p[2*i]-huil_p[0], p[2*i+1]-huil_p[1]])
        b_extra[i] = alpha*cbf_h(np.array([p[2*i],p[2*i+1]]), huil_p, d_extra, -1) - 2*(p[2*i]-huil_p[0])*vxe - 2*(p[2*i+1]-huil_p[1])*vye

    #----------------------------
    # Solve minimization problem
    #----------------------------
    #Define linear constraints
    constraint_cm = LinearConstraint(A_cm*cm, lb=-b_cm*cm, ub=np.inf)
    constraint_oa = LinearConstraint(A_oa*oa, lb=-b_oa*oa, ub=np.inf)
    constraint_arena = LinearConstraint(A_arena, lb=-b_arena, ub=np.inf)
    constraint_extra = LinearConstraint(A_extra, lb=-b_extra, ub=np.inf)
    
    #Define objective function
    def objective_function(u, u_n):
        return np.linalg.norm(u - u_n)**2
    
    #Construct the problem
    u = minimize(
        objective_function,
        x0=u_n,
        args=(u_n,),
        constraints=[constraint_cm, constraint_oa, constraint_arena, constraint_extra],
    )

    return u.x

def systemDynamics(p, u):
    # System dynamics parameters
    f = np.zeros(len(p))
    g = np.identity(len(u))

    # Update state vector derivative
    xdot = f+np.dot(g,u)
    return xdot