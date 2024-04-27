#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 19:09:42 2024

@author: mrityunjay
"""
import numpy as np
from matplotlib.path import Path
from scipy.optimize import linprog

def is_near_goal(pos_a, pos_b, tolerance=2):
    return np.allclose(pos_a, pos_b, atol=tolerance)

# Convert from (-pi,pi) to (0,2pi)
def convert_to_0_2pi(angle):
    if angle < 0:
        return 2*np.pi + angle
    return angle

def ssa(angle):
    if angle > np.pi:
        angle = angle - 2*np.pi
    return angle

def ssa_array(angle):
    
    angle = angle % (2*np.pi)

    angle = np.where(angle > np.pi, angle - 2*np.pi, angle)
    
    return angle

def pidcontroller_vo(U, y, r, control_dict1):
    kp1 = control_dict1['kp1']
    kd1 = control_dict1['kd1']
    n = len(U)
    delta_sat = 35*np.pi/180
    tau = np.zeros((n,3))
    V = np.array([[1,0,0,0,0,0,0,0,0,0],
                  [0,1,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,1,0,0,0,0]])
    
    W = np.array([[0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0,0,0]]) 
    z = y
    z[2] = r
    for i in range(n):
        e = U[i] - np.dot(V, y)
        e[2] = np.mod(e[2], 2*np.pi)
        e[2] = np.where(e[2] > np.pi, e[2]-2*np.pi , e[2])
        tau[i] = kp1 @ e + kd1 @ (np.dot(W, z))
        tau[i][0] = np.linalg.norm(tau[i][0])
        tau[i][2] = np.where(abs(tau[i][2]) > delta_sat, np.sign(tau[i][2])*delta_sat, tau[i][2])
    return tau

def saturate(x, a):
    if abs(x) > a:
        x = np.sign(x) * a

    return float(x)

def control(psid, ss, control_dict=None):
    
    # n_c = 115.5 / 60  # commanded propeller speed (1:75.5 scaled KCS to achieve 1.42 m/s in steady state)
    psi = ss[5]
    r = ss[2]
    
    u = ss[0]
    v = ss[1]
    s_err = ss[9]
    s = np.sqrt(u **2 + v **2)
    n_c = -3*(s - 1) - 0.1 * s_err 
    
    if control_dict['control_type'] == 'PD':
        
        Kp = control_dict['Kp']
        Kd = control_dict['Kd']
        
        e = ssa(psi - psid)                             # error
        delta_c = -Kp * e - Kd * r                      # commanded rudder angle
        delta_c = saturate(delta_c, 35*np.pi/180)       # in radians
    
    return delta_c, n_c


def wp_switch(state, waypoints, latest_tracked=0):
    x_curr , y_curr = state[3], state[4]
    r = np.sqrt((x_curr-waypoints[latest_tracked][0])**2+ (y_curr-waypoints[latest_tracked][1])**2)
    if r < 5:
        latest_tracked += 1
    return latest_tracked

def ilos_guidance(ss, gwp=None, iwp=None, dLA=1, ki=0, kappa=0):
    
    x_own = ss[3]
    y_own = ss[4]
    psi = ss[5]
    yp_int = ss[8]      # For implementing ILOS
    
    xg = gwp[0]         # Goal waypoint
    yg = gwp[1]
    
    xi = iwp[0]         # Initial waypoint
    yi = iwp[1]
    
    pi_p = np.arctan2(yg - yi, xg - xi)

    # For calculating yp_e
    R_np = np.array([[np.cos(pi_p), -np.sin(pi_p)], 
                     [np.sin(pi_p), np.cos(pi_p)]])
    
    # err (2*1 vector) contains xpe and ype
    err = np.transpose(R_np) @ np.array([[x_own - xi], [y_own - yi]])
    goal = np.transpose(R_np) @ np.array([[xg - xi], [yg - yi]])

    xpe, ype = err                                              # along track error # cross track error
    xge, yge = goal
    
    kp = 1/dLA
    # psi_des = pi_p - np.arctan(kp*ype + ki*yp_int)
    if xpe < xge:
        psi_des = pi_p - np.arctan(kp*ype + ki*yp_int)              # desired heading angle
    else:
        psi_des = pi_p - np.pi + np.arctan(kp*ype + ki*yp_int)      # desired heading angle
    
    ypd_int = (dLA*ype)/(dLA**2 + (ype + kappa*yp_int)**2)      # rate of change of cross track error
        
    return psi_des, ypd_int, xpe, ype

# Mapping into the Velocity Space from Configuration Space 
def reactive_guidance(wpf, obstacles, guidance_dict, boundary_grad = None):

    collision_flag = 0
    collision_tol = guidance_dict['collision_tolerance']

    grad = np.zeros(2)
    
    
    if obstacles:

        Xg = np.array(wpf)
        Xn = np.array([obstacles[0].agent_x, obstacles[0].agent_y])   
        
        if guidance_dict['reactive_guidance'] == 'vo': 
            
            VO = []
            
            for obs in obstacles:
            
                Xo = np.array([obs.x, obs.y])
        
                # Check for collision with obstacle
                if np.linalg.norm(Xn - Xo) < collision_tol:
                    collision_flag = 1
        
                R_tol = guidance_dict['collision_tolerance']
        
                dij = Xo - Xn
                dij_norm = np.linalg.norm(dij)
                dij_p = dij / dij_norm
                
                u1 = obs.agent_u
                v1 = obs.agent_v
                psi1 = obs.agent_psi
            
                u2 = obs.u
                v2 = obs.v
                psi2 = obs.psi
                
                u1_glob = u1*np.cos(psi1) - v1*np.sin(psi1)
                v1_glob = u1*np.sin(psi1) + v1*np.cos(psi1)
                
                u2_glob = u2*np.cos(psi2) - v2*np.sin(psi2)
                v2_glob = u2*np.sin(psi2) + v2*np.cos(psi2)                
            
                vj = np.array([u2_glob, v2_glob])
                vi = np.array([u1_glob, v1_glob])
                vr = vi - vj
                
                #Construction of the Velocity Cone
                apex = vj                              # VO
                x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi1 = np.arctan2(x1[1],x1[0])
                theta_left = phi1
                l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                l1_norm = np.linalg.norm(l1)
                l1_p = l1 / l1_norm
                left_pt = apex + l1
                
                x2 = (apex + dij + (2*R_tol) * np.array([dij_p[1], -dij_p[0]]))
                phi2 = np.arctan2(x2[1],x2[0])
                theta_right = phi2
                l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                l2_norm = np.linalg.norm(l2)
                l2_p = l2 /l2_norm
                right_pt = apex + l2
                
                VO.append([apex, left_pt, right_pt, dij, vr])
            
            psi = reachable_velocities(vi, VO, guidance_dict)
                
        elif guidance_dict['reactive_guidance'] == 'rvo': 
            
            VO = []
            for obs in obstacles:
            
                Xo = np.array([obs.x, obs.y])
        
                # Check for collision with obstacle
                if np.linalg.norm(Xn - Xo) < collision_tol:
                    collision_flag = 1
        
                R_tol = guidance_dict['collision_tolerance']
        
                dij = Xo - Xn
                dij_norm = np.linalg.norm(dij)
                dij_p = dij / dij_norm
                
                u1 = obs.agent_u
                v1 = obs.agent_v
                psi1 = obs.agent_psi
            
                u2 = obs.u
                v2 = obs.v
                psi2 = obs.psi
                
                u1_glob = u1*np.cos(psi1) - v1*np.sin(psi1)
                v1_glob = u1*np.sin(psi1) + v1*np.cos(psi1)
                
                u2_glob = u2*np.cos(psi2) - v2*np.sin(psi2)
                v2_glob = u2*np.sin(psi2) + v2*np.cos(psi2)                
            
                vj = np.array([u2_glob, v2_glob])
                vi = np.array([u1_glob, v1_glob])
                vr = vi - vj
                
                #Construction of the Velocity Cone
                apex = 0.5 * vj + 0.5 * vi                  # RVO
                x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi1 = np.arctan2(x1[1],x1[0])
                theta_left = phi1
                l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                l1_norm = np.linalg.norm(l1)
                l1_p = l1 / l1_norm
                left_pt = apex + l1
                
                x2 = (apex + dij + (2*R_tol)* np.array([dij_p[1], -dij_p[0]]))
                phi2 = np.arctan2(x2[1],x2[0])
                theta_right = phi2
                l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                l2_norm = np.linalg.norm(l2)
                l2_p = l2 /l2_norm
                right_pt = apex + l2
                
                VO.append([apex, left_pt, right_pt, dij, vr]) 
        
            psi = reachable_velocities(vi, VO, guidance_dict)
            
        elif guidance_dict['reactive_guidance'] == 'hrvo': 
            
            VO = []
            for obs in obstacles:
            
                Xo = np.array([obs.x, obs.y])
        
                # Check for collision with obstacle
                if np.linalg.norm(Xn - Xo) < collision_tol:
                    collision_flag = 1
        
                R_tol = guidance_dict['collision_tolerance']
        
                dij = Xo - Xn
                dij_norm = np.linalg.norm(dij)
                dij_p = dij / dij_norm
                
                u1 = obs.agent_u
                v1 = obs.agent_v
                psi1 = obs.agent_psi
            
                u2 = obs.u
                v2 = obs.v
                psi2 = obs.psi
                
                u1_glob = u1*np.cos(psi1) - v1*np.sin(psi1)
                v1_glob = u1*np.sin(psi1) + v1*np.cos(psi1)
                
                u2_glob = u2*np.cos(psi2) - v2*np.sin(psi2)
                v2_glob = u2*np.sin(psi2) + v2*np.cos(psi2)                
            
                vj = np.array([u2_glob, v2_glob])
                vi = np.array([u1_glob, v1_glob])
                vr = vi - vj
                
                #Construction of the Velocity Cone
                
                apex1 = vj                              # VO
                x11 = (apex1 + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi11 = np.arctan2(x11[1],x11[0])
                theta_left1 = phi11
                l11 = dij_norm * np.array([np.cos(theta_left1), np.sin(theta_left1)])
                l11_norm = np.linalg.norm(l11)
                l11_p = l11 / l11_norm
                left_pt1 = apex1 + l11
                
                x12 = (apex1 + dij + (2*R_tol) * np.array([dij_p[1], -dij_p[0]]))
                phi12 = np.arctan2(x12[1],x12[0])
                theta_right1 = phi12
                l12 = dij_norm * np.array([np.cos(theta_right1), np.sin(theta_right1)])
                l12_norm = np.linalg.norm(l12)
                l12_p = l12 /l12_norm
                right_pt1 = apex1 + l12
                
                apex2 = 0.5 * vj + 0.5 * vi                  # RVO
                x21 = (apex2 + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi21 = np.arctan2(x21[1],x21[0])
                theta_left2 = phi21
                l21 = dij_norm * np.array([np.cos(theta_left2), np.sin(theta_left2)])
                l21_norm = np.linalg.norm(l21)
                l21_p = l21 / l21_norm
                left_pt2 = apex2 + l21
                
                x22 = (apex2 + dij + (2*R_tol) * np.array([dij_p[1], -dij_p[0]]))
                phi22 = np.arctan2(x22[1],x22[0])
                theta_right2 = phi22
                l22 = dij_norm * np.array([np.cos(theta_right2), np.sin(theta_right2)])
                l22_norm = np.linalg.norm(l22)
                l22_p = l22 /l22_norm
                right_pt2 = apex2 + l22
                
                l = apex2 + dij_norm*dij_p
                
                Rn = np.array([[np.cos(np.pi/2), np.sin(np.pi/2)],[-np.sin(np.pi/2), np.cos(np.pi/2)]])
                l_n = np.dot(Rn, l)
                l_np = l_n / np.linalg.norm(l_n)
                dot_p = np.dot(vi, l_np)

                if dot_p > apex2[0]:
                    slope1 = x22[1] / x22[0]  
                    intercept1 = x22[1] - slope1 * x22[0]  
                    
                    slope2 = x11[1] / x11[0]  
                    intercept2 = x11[1] - slope2 * x11[0]  
                    
                    x_ = (intercept2 - intercept1) / (slope1 - slope2)
                    y_ = slope1 * x_ + intercept1
                    
                    apex = np.array([x_, y_])                      # HRVO
                    
                    x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                    phi1 = np.arctan2(x1[1],x1[0])
                    theta_left = phi1
                    l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                    l1_norm = np.linalg.norm(l1)
                    l1_p = l1 / l1_norm
                    left_pt = apex + l1
                    
                    x2 = (apex + dij + (2*R_tol)* np.array([dij_p[1], -dij_p[0]]))
                    phi2 = np.arctan2(x2[1],x2[0])
                    theta_right = phi2
                    l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                    l2_norm = np.linalg.norm(l2)
                    l2_p = l2 /l2_norm
                    right_pt = apex + l2
                    
                elif dot_p < apex2[0]:
                    slope1 = x21[1] / x21[0]  
                    intercept1 = x21[1] - slope1 * x21[0]  
                    
                    slope2 = x12[1] / x12[0]  
                    intercept2 = x12[1] - slope2 * x12[0]  
                    
                    x_ = (intercept2 - intercept1) / (slope1 - slope2)
                    y_ = slope1 * x_ + intercept1
                    
                    apex = np.array([x_, y_])                      # HRVO
                    
                    x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                    phi1 = np.arctan2(x1[1],x1[0])
                    theta_left = phi1
                    l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                    l1_norm = np.linalg.norm(l1)
                    l1_p = l1 / l1_norm
                    left_pt = apex + l1
                    
                    x2 = (apex + dij + (2*R_tol)* np.array([dij_p[1], -dij_p[0]]))
                    phi2 = np.arctan2(x2[1],x2[0])
                    theta_right = phi2
                    l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                    l2_norm = np.linalg.norm(l2)
                    l2_p = l2 /l2_norm
                    right_pt = apex + l2
                else:
                    #Construction of the Velocity Cone
                    apex = 0.5 * vj + 0.5 * vi                  # RVO
                    x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                    phi1 = np.arctan2(x1[1],x1[0])
                    theta_left = phi1
                    l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                    l1_norm = np.linalg.norm(l1)
                    l1_p = l1 / l1_norm
                    left_pt = apex + l1
                    
                    x2 = (apex + dij + (2*R_tol)* np.array([dij_p[1], -dij_p[0]]))
                    phi2 = np.arctan2(x2[1],x2[0])
                    theta_right = phi2
                    l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                    l2_norm = np.linalg.norm(l2)
                    l2_p = l2 /l2_norm
                    right_pt = apex + l2
                
                VO.append([apex, left_pt, right_pt, dij, vr]) 
        
            psi = reachable_velocities(vi, VO, guidance_dict)
            
        elif guidance_dict['reactive_guidance'] == 'orca': 
            
            VO = []
            for obs in obstacles:
            
                Xo = np.array([obs.x, obs.y])
        
                # Check for collision with obstacle
                if np.linalg.norm(Xn - Xo) < collision_tol:
                    collision_flag = 1
        
                R_tol = guidance_dict['collision_tolerance']
        
                dij = Xo - Xn
                dij_norm = np.linalg.norm(dij)
                dij_p = dij / dij_norm
                
                u1 = obs.agent_u
                v1 = obs.agent_v
                psi1 = obs.agent_psi
            
                u2 = obs.u
                v2 = obs.v
                psi2 = obs.psi
                
                u1_glob = u1*np.cos(psi1) - v1*np.sin(psi1)
                v1_glob = u1*np.sin(psi1) + v1*np.cos(psi1)
                
                u2_glob = u2*np.cos(psi2) - v2*np.sin(psi2)
                v2_glob = u2*np.sin(psi2) + v2*np.cos(psi2)                
            
                vj = np.array([u2_glob, v2_glob])
                vi = np.array([u1_glob, v1_glob])
                vr = vi - vj
                
                #Construction of the Velocity Cone
                #apex = vj
                apex = 0.5 * vj + 0.5 * vi                  # RVO
                x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi1 = np.arctan2(x1[1],x1[0])
                theta_left = phi1
                l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                l1_norm = np.linalg.norm(l1)
                l1_p = l1 / l1_norm
                left_pt = apex + l1
                
                x2 = (apex + dij + (2*R_tol)* np.array([dij_p[1], -dij_p[0]]))
                phi2 = np.arctan2(x2[1],x2[0])
                theta_right = phi2
                l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                l2_norm = np.linalg.norm(l2)
                l2_p = l2 /l2_norm
                right_pt = apex + l2
                
                VO.append([apex, left_pt, right_pt, dij, vr]) 
        
            psi = reachable_velocities(vi, VO, guidance_dict)
    
    return psi , collision_flag        

#Reachable Avoidance Velocity (RAV) 
def reachable_velocities(vel_d, VO, guidance_dict):
    v = np.array([2,2])
    v_norm = np.linalg.norm(v)
    num_points1 = 100
    num_points2 = 10
    v_new = []
    cone_paths = []
        
    #Velocity Cone (VO)       
    for vo in VO:
        apex, left_boundary, right_boundary, _, _ = vo
        cone_path_data = [
            (Path.MOVETO, apex),
            (Path.LINETO, right_boundary),
            (Path.LINETO, left_boundary),
            (Path.CLOSEPOLY, apex),  
        ]
        
        cone_path = Path([vertex for code, vertex in cone_path_data], [code for code, _ in cone_path_data])
        cone_paths.append(cone_path)
    
    #Reachable Velocity (RV) 
    for th in np.linspace(-np.pi, np.pi, num_points1):
        for mag in np.linspace(0.05, v_norm + 0.05, num_points2):
            new_vel = [mag * np.cos(th) , mag * np.sin(th)]
            
            v_outside = True
            for vo, cone_path in zip(VO, cone_paths):
                apex, left_boundary, right_boundary, dij, vr= vo   
                if cone_path.contains_point(new_vel):
                    v_outside = False
                    break
                
            if v_outside:
                v_new.append(new_vel)
                    
    v_new = np.array(v_new)

    Vopt_A = min(v_new, key=lambda v: cost_function1(v, vel_d)) 
    psi = np.arctan2(Vopt_A[1], Vopt_A[0])
    if guidance_dict['reactive_guidance'] == 'orca':
        u = np.linalg.norm(Vopt_A) - vr
        w = u + vr
        n = w / np.linalg.norm(w)
        V = np.dot((Vopt_A - (vel_d + 0.5 * u)), n)
        V_norm = np.linalg.norm(V)
        if V_norm > 0:       
            psi = np.arctan2(Vopt_A[1], Vopt_A[0])
        
            dij_norm = np.linalg.norm(dij)
            l = dij_norm * np.array([np.cos(-psi), np.sin(-psi)])
            
            #ColREGS Implemented
            if np.cross(dij, vr) < 0:
                proj_mat = np.outer(l, l) / np.inner(l, l)
                Vopt_A = np.dot(proj_mat, Vopt_A)
                psi = np.arctan2(Vopt_A[1], Vopt_A[0])
            return psi
    
    # dij_norm = np.linalg.norm(dij)
    # l = dij_norm * np.array([np.cos(-psi), np.sin(-psi)])
    
    # #ColREGS Implemented
    # if np.cross(dij, vr) < 0:
    #     proj_mat = np.outer(l, l) / np.inner(l, l)
    #     Vopt_A = np.dot(proj_mat, Vopt_A)
    #     psi = np.arctan2(Vopt_A[1], Vopt_A[0])
    return psi
    

# #Reachable Avoidance Velocity (RAV) 
# def reachable_velocities(vel_d, VO):
#     v = np.array([2,2])
#     v_norm = np.linalg.norm(v)
#     num_points1 = 100
#     num_points2 = 10
#     v_new = []
#     cone_paths = []
        
#     #Velocity Cone (VO)       
#     for vo in VO:
#         apex, left_boundary, right_boundary, _, _ = vo
#         cone_path_data = [
#             (Path.MOVETO, apex),
#             (Path.LINETO, right_boundary),
#             (Path.LINETO, left_boundary),
#             (Path.CLOSEPOLY, apex),  
#         ]
        
#         cone_path = Path([vertex for code, vertex in cone_path_data], [code for code, _ in cone_path_data])
#         cone_paths.append(cone_path)
    
#     #Reachable Velocity (RV) 
#     for th in np.linspace(-np.pi, np.pi, num_points1):
#         for mag in np.linspace(0.05, v_norm + 0.05, num_points2):
#             new_vel = [mag * np.cos(th) , mag * np.sin(th)]
            
#             v_outside = True
#             for vo, cone_path in zip(VO, cone_paths):
#                 apex, left_boundary, right_boundary, dij, vr= vo    
#                 if cone_path.contains_point(new_vel):
#                     v_outside = False
#                     break
                
#             if v_outside:
#                 v_new.append(new_vel)
                    
#     v_new = np.array(v_new)

#     Vopt_A = min(v_new, key=lambda v: cost_function1(v, vel_d)) 
#     psi = np.arctan2(Vopt_A[1], Vopt_A[0])
#     dij_norm = np.linalg.norm(dij)
#     l = dij_norm * np.array([np.cos(-psi), np.sin(-psi)])
    
#     #ColREGS Implemented
#     if np.cross(dij, vr) < 0:
#         proj_mat = np.outer(l, l) / np.inner(l, l)
#         Vopt_A = np.dot(proj_mat, Vopt_A)
#     psi = np.arctan2(Vopt_A[1], Vopt_A[0])
#     return psi

def cost_function1(v, vel_d):
    return np.linalg.norm(np.array(v) - np.array(vel_d))
