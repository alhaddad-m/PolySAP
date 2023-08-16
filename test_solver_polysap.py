import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time
import math
import yaml
import create_solver_polysap
from ctypes import sizeof

acados_solver = create_solver_polysap.Solver()
def test_solver(x_ref_points , y_ref_points , theta_initial ,x_edge , y_edge):
    print("START SOLVING")
    factor = 1
    nx = 4
    nu = 2
    ny = nx + nu
    N = 30

    t = time.time()
    elapsed = 1000 * (time.time() - t)
    yref = np.zeros([N,ny +1])
    theta_0 = theta_initial
    v_0 = 0
    v_e = 0
    x_ref = []
    y_ref = []
    theta_segments = []
    theta_ref = []
    init_x = []
    init_y = []
    init_theta = []
    N = 30
    len_segments = []
    theta_segments = np.append(theta_segments , theta_0 ) # current orientation robot
    theta_ref = np.append(theta_ref , theta_0 )
    num_segment = len(x_ref_points)-1
    length_path = 0
    for i in range(num_segment):
        length_path = length_path + math.sqrt((x_ref_points[i+1]-x_ref_points[i])**2+(y_ref_points[i+1]-y_ref_points[i])**2)
        theta_middle = math.atan2(y_ref_points[i+1]-y_ref_points[i], x_ref_points[i+1]-x_ref_points[i])
        if(theta_0>1.57 and theta_middle<0):
          theta_middle = 6.28 + theta_middle
        theta_segments = np.append(theta_segments , theta_middle)
        x_center_segment = (x_ref_points[i+1]+x_ref_points[i])/2
        y_center_segment = (y_ref_points[i+1]+y_ref_points[i])/2 
        print("segment center" , x_center_segment , y_center_segment)
        len_segments = np.append(len_segments , math.sqrt((x_ref_points[i+1]-x_ref_points[i])**2+(y_ref_points[i+1]-y_ref_points[i])**2))

    step_line = length_path / N
    theta_e = theta_segments[-1]
    print("theta_e",theta_e)
    print(length_path)
    k = 0
    x_ref = np.append(x_ref , x_ref_points[0])
    y_ref = np.append(y_ref , y_ref_points[0])
    for i in range(N+1):
        x_ref = np.append(x_ref , x_ref[i] + step_line * math.cos(theta_segments[k+1]))
        y_ref = np.append(y_ref , y_ref[i] + step_line * math.sin(theta_segments[k+1]))
        theta_ref = np.append(theta_ref , theta_segments[k+1])
        d = math.sqrt((x_ref[-1]-x_ref_points[k])**2+(y_ref[-1]-y_ref_points[k])**2)
        if(d>len_segments[k] and k<(num_segment-1)):
            k = k+1
            x_ref[i] = x_ref_points[k]
            y_ref[i] = y_ref_points[k]
        elif (k>(num_segment-1)):
            break
    x0 = np.array([x_ref_points[0],y_ref_points[0],v_0,theta_0])
    init_x = x_ref[0:N+1]
    init_y = y_ref[0:N+1]
    init_theta = theta_ref[0:N+1]
    x_goal = np.array([init_x[-1],init_y[-1],v_e,theta_e])
    print("x_goal" , x_goal)
        
    v_d = 0.2

    new_time_step = ((length_path)/(v_d*N)) * np.ones(N)     

    num_edge = 100
    parameter_values = 100*np.ones(4*num_edge)
    parameter_values[0:len(x_edge)] = x_edge
    parameter_values[int(num_edge*2):len(y_edge)+ int(num_edge*2)] = y_edge

    print("Parameter Values" , parameter_values)

    yref[:,0]=init_x[0:N]
    yref[:,1]=init_y[0:N]
    yref[:,2]=1
    yref[:,3] = init_theta[0:N]
    print(init_theta)
    a = np.zeros(1)
    yref_e = np.concatenate([x_goal,a])  
    x_traj_init = np.transpose([ yref[:,0] , yref[:,1] , yref[:,2] , yref[:,3] ])

    for i in range(30):
        print("YREF" , yref[i,0],yref[i,1],yref[i,3])

    simX = np.zeros((N+1, 4))
    simU = np.zeros((N, 2))
 
    for i in range(N):
        acados_solver.set(i,'p',parameter_values)
        acados_solver.set(i,'y_ref',yref[i])
        acados_solver.set(i, 'x', x_traj_init[i])
        acados_solver.set(i, 'u', np.array([0.0, 0.0]))
    acados_solver.set(N, 'p',  parameter_values)
    acados_solver.set(N, 'y_ref', yref_e)
    acados_solver.set(N, 'x', x_goal)
    acados_solver.set(0,'lbx', x0)
    acados_solver.set(0,'ubx', x0)
    acados_solver.options_set('rti_phase', 0)
    acados_solver.set_new_time_steps(new_time_step)

    t = time.time()
    status = acados_solver.solve()
    print("status" , status)
    elapsed = 1000 * (time.time() - t)
    for i in range(N + 1):
        x = acados_solver.get(i, "x")
        simX[i,:]=x
    cost = acados_solver.get_cost()
    for i in range(N):
        u = acados_solver.get(i, "u")
        simU[i,:]=u
   
    print("Elapsed time: {} ms".format(elapsed))

    return simX , init_x , init_y
      
fig, ax2 = plt.subplots(figsize=(6, 6))
fig.tight_layout()

## polygon Shanghae
x_edge = np.array([4.62,6.7,6.7,9.56,9.56,5.06,5.06,4.62,5.38,8.8,8.8,9.18,9.18,5.92,5.92,5.38,13.36,11.48,11.48,9.82,9.82,12.76,12.76,13.36,13.36,13.36,10.94,9.8,9.8,10.38,10.38,11.46,11.46,10.94,1.72,1.1,1.1,1.22,1.22,1.98,1.98,1.72,2.1,1.9,1.9,2.38,2.38,2.54,2.54,2.1,13.28,10.74,10.74,13.36,13.36,13.28,2.68,2.44,2.44,3.02,3.02,3.2,3.2,2.68,4.62,4.18,4.18,2.8,2.8,3.2,3.2,4.6,4.6,4.62,5.56,4.32,4.32,4.58,4.58,5.88,5.88,5.56,10,4.62,4.62,6.24,6.24,11.44,11.44,10,9.66,8.46,8.46,8.76,8.76,9.98,9.98,9.66,13.36,11.92])
y_edge = np.array([0.98,0.04,0.04,0.04,0.04,1.86,1.86,0.98,1.92,0.54,0.54,1.14,1.14,2.74,2.74,1.92,2.48,3.34,3.34,0.04,0.04,0.04,0.04,1.28,1.28,2.48,3.58,1.18,1.18,0.86,0.86,3.32,3.32,3.58,4.58,3.72,3.72,3.02,3.02,4.4,4.4,4.58,5.76,5.46,5.46,5.18,5.18,5.46,5.46,5.76,7.76,5.54,5.54,3.7,3.7,7.76,7.88,6.94,6.94,6.54,6.54,7.82,7.82,7.88,9.54,9.64,9.64,8.22,8.22,7.84,7.84,9.16,9.16,9.54,6.08,4.72,4.72,4.46,4.46,5.82,5.82,6.08,9.94,4.44,4.44,2.96,2.96,8.54,8.54,9.94,10.26,9.06,9.06,8.68,8.68,9.96,9.96,10.26,12.52,11.38])
x_ref_points = [[11.96 ,10.36 , 11.83, 11.23]]
y_ref_points = [[3.95,5.63,8.49,9.9]]
theta_start = 2
for i in range(0,len(x_edge),2):
    ax2.plot([x_edge[i] , x_edge[i+1]], [y_edge[i] , y_edge[i+1]], color='g')

simplot2 = np.zeros((len(x_ref_points)*31, 2))
siminitial2 = np.zeros((len(x_ref_points)*31, 2))
theta_initial = theta_start
for i in range(len(x_ref_points)):
    Simx , init_x , init_y = test_solver(x_ref_points[i] , y_ref_points[i], theta_initial , x_edge , y_edge)
    if(i < (len(x_ref_points)-1)):
      x_ref_points[i+1][0] = Simx[-1,0]
      y_ref_points[i+1][0] = Simx[-1,1]
    theta_initial = Simx[-1,3]
    for j in range(31):
        simplot2[i*31 + j,0] = Simx[j,0]
        simplot2[i*31 + j,1] = Simx[j,1]
        siminitial2[i*31 + j,0] = init_x[j]
        siminitial2[i*31 + j,1] = init_y[j]
ax2.plot(simplot2[:, 0], simplot2[:, 1] , linewidth=2, label='PolySAP Path')#,marker='o')
ax2.plot(siminitial2[:, 0], siminitial2[:, 1] , linewidth=2, linestyle='dashed', label='Initial Guess')
ax2.set_aspect('equal', 'box')
plt.xlim(0, 14)
ax2.legend()
asp = np.diff(ax2.get_xlim())[0] / np.diff(ax2.get_ylim())[0]
ax2.set_aspect(asp)
plt.show()

