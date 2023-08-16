from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan, exp, if_else, pi , atan , logic_and , sqrt , fabs , atan2
import numpy as np
import math 


def robot_model():

    model_name = "robot_model"

    # State
    x = SX.sym('x') 
    y = SX.sym('y')   
    v = SX.sym('v')  
    theta = SX.sym('theta') 

    sym_x = vertcat(x, y, v ,theta)

    # Input
    a = SX.sym('a')
    w = SX.sym('w')
    sym_u = vertcat(a, w)

    # Derivative of the States
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    v_dot = SX.sym('v_dot')
    x_dot = vertcat(x_dot, y_dot, v_dot, theta_dot)

    ## Model of Robot
    f_expl = vertcat(sym_x[2] * cos(sym_x[3]),
                        sym_x[2] * sin(sym_x[3]),
                        sym_u[0],
                        sym_u[1])
    f_impl = x_dot - f_expl

    model = AcadosModel()

    num_edge = 100
    
    # parameters vector 
    obst_param = SX.sym('p', 4*num_edge) 
    sym_p = vertcat(obst_param)

    

    obst = SX.sym('obst', num_edge)

    tri_X = obst_param[0:2*num_edge]
    tri_Y = obst_param[2*num_edge:4*num_edge]
    print(tri_X)
    print("TriY")
    print(tri_Y)

    i = 0
    for j in range(num_edge):
        d_x = tri_X[i+1] - tri_X[i]
        d_y = tri_Y[i+1] - tri_Y[i]
        norm_ = sqrt((tri_X[i] - tri_X[i+1])**2 + (tri_Y[i] - tri_Y[i+1])**2)
        cos_phi = d_x / norm_
        sin_phi = d_y / norm_
        C1 = (tri_X[i+1] - tri_X[i])*cos_phi + (tri_Y[i+1] - tri_Y[i])*sin_phi  
        x0 = (x - tri_X[i])*cos_phi + (y - tri_Y[i])*sin_phi
        y0 = - (x - tri_X[i])*sin_phi + (y - tri_Y[i])*cos_phi
        sigma = 0.3
      
        cond1 = y0 + 0.5 > 0
        cond2 = y0 < 0 
        cond3 = x0 < (C1+0.3) 
        cond4 = x0 > -0.3
        cond5 = logic_and(cond1 , cond2)
        cond6 = logic_and(cond3 , cond4)
        cond7 = logic_and(cond5 , cond6)
        cost_sem = (1/(sigma*sqrt(2*pi)))*exp(-(y0)**2/(2*sigma**2))

        
        obst[j] = if_else(cond7 , 4*cost_sem ,0)  
        i = i+2


    obst_all = 0
    for i in range(num_edge):
        obst_all = obst_all + obst[i]


    model.cost_y_expr = vertcat(sym_x, sym_u ,obst_all)
    model.cost_y_expr_e = vertcat(sym_x ,obst_all) 
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = sym_x
    model.xdot = x_dot
    model.u = sym_u
    model.p = sym_p
    model.name = model_name

    return model