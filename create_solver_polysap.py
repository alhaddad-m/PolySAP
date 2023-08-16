from acados_template import AcadosOcp, AcadosOcpSolver
from robot_model_polysap import robot_model
import numpy as np
import time
import math
import yaml


def Solver():
    # acados OCP handle
    model = robot_model()
    ocp = AcadosOcp()
    N = 30
    ocp.dims.N = N
    # OCP dimensions
    nx = 4
    nu = 2
    ny = nx + nu
    # OCP costs
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.model.cost_y_expr = model.cost_y_expr
    ocp.model.cost_y_expr_e = model.cost_y_expr_e

    w_x = 5
    w_y = 5
    w_v = 0.00005 
    w_theta = 0.001
    w_a = 0.01
    w_w = 0.000001 
    w_x_e = 10 
    w_y_e = 10
    w_v_e = 0.01 
    w_theta_e = 0.01 
    W_obst = np.array([10])

    W_x = np.array([w_x, w_y, w_v, w_theta, w_a, w_w])
    W = np.diag(np.concatenate([W_x,W_obst]))
    ocp.cost.W = W
    W_xe = np.array([w_x_e, w_y_e, w_v_e, w_theta_e])
    W_e = np.diag(np.concatenate([W_xe,W_obst]))
    ocp.cost.W_e = W_e   
    ocp.cost.yref =  np.zeros([ny+1])
    ocp.cost.yref_e = np.zeros([nx+1])
    
    ocp.constraints.idxbx = np.array([0, 1, 2 , 3 ])
    ocp.constraints.lbx = np.array([-100, -100, 0 , -100])
    ocp.constraints.ubx = np.array([100, 100, 1 , 100])
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([-0.05, -0.3])
    ocp.constraints.ubu = np.array([0.05, 0.3])

    num_edge = 100
    paramters = 100*np.ones(4*num_edge) 


    ocp.parameter_values = paramters
                                
    x0 = np.array([0, 0, 0, 0])
    ocp.constraints.x0 = x0

    ocp.model = model

    ocp.solver_options.tf = 12
    ocp.solver_options.qp_solver =  'PARTIAL_CONDENSING_HPIPM' 
    ocp.solver_options.qp_solver_cond_N = 10
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.levenberg_marquardt = 3.0
    ocp.solver_options.nlp_solver_max_iter = 15
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.nlp_solver_tol_stat = 1e2
    ocp.solver_options.nlp_solver_tol_eq = 1e-1
    ocp.solver_options.print_level = 0

    acados_solver = AcadosOcpSolver(ocp, json_file="acados_solver_polysap.json")

    return acados_solver


Solver()
print("Acados solver was generated")