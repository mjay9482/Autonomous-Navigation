#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 19:09:42 2024

@author: mrityunjay
"""
import numpy as np
from matplotlib.path import Path
from scipy.optimize import linprog
import env

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
    n_c = -3 * (s - 1) - 0.1 * s_err 
    if control_dict['control_type'] == 'PD':
        
        Kp = control_dict['Kp']
        Kd = control_dict['Kd']
        
        e = ssa(psi - psid)                             # error
        delta_c = -Kp * e - Kd * r                      # commanded rudder angle
        delta_c = saturate(delta_c, 35*np.pi/180)       # in radians

    return delta_c, n_c

def ilos_guidance(ss, gwp=None, iwp=None, dLA=1, ki=0, kappa=0):
    
    x_own = ss[3]
    y_own = ss[4]
    psi = ss[5]
    yp_int = ss[8]      # For implementing ILOS
    
    xg = gwp[0]         # Goal waypoint
    yg = gwp[1]
    
    xi = ss[3]
    yi = ss[4]
    # xi = iwp[0]         # Initial waypoint
    # yi = iwp[1]
    
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
                apex = vj                                                                 # VO
                x1 = (apex + dij  + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi1 = np.arctan2(x1[1], x1[0])
                
                theta_left = phi1 
                l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                l1_norm = np.linalg.norm(l1)
                l1_p = l1 / l1_norm
                left_ray = apex + l1
                
                
                x2 = (apex + dij + (2*R_tol) * np.array([dij_p[1], -dij_p[0]]))
                phi2 = np.arctan2(x2[1], x2[0])
                
                theta_right = phi2 
                l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                l2_norm = np.linalg.norm(l2)
                l2_p = l2 /l2_norm
                right_ray = apex + l2
                
                psir = np.abs(psi2- psi1)
                bt = np.arctan2(dij[1], dij[0])
                br = bt - psir
                
                VO.append([apex, left_ray, right_ray, dij, vr, br, psir, Xn, Xo, vi, vj])
            
            psi = reachable_velocities(vi, VO, wpf, guidance_dict, Xo, Xn)[0]

                
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
                apex = 0.5*vj   + 0.5*vi                              # RVO
                x1 = (apex + dij + (2*R_tol) * np.array([-dij_p[1], dij_p[0]]))
                phi1 = np.arctan2(x1[1],x1[0])
                
                theta_left = phi1
                l1 = dij_norm * np.array([np.cos(theta_left), np.sin(theta_left)])
                l1_norm = np.linalg.norm(l1)
                l1_p = l1 / l1_norm
                left_ray = apex + l1
                
                x2 = (apex + dij + (2*R_tol)* np.array([dij_p[1], -dij_p[0]]))
                phi2 = np.arctan2(x2[1],x2[0])
                
                theta_right = phi2
                l2 = dij_norm * np.array([np.cos(theta_right), np.sin(theta_right)])
                l2_norm = np.linalg.norm(l2)
                l2_p = l2 /l2_norm
                right_ray = apex + l2
                
                psir = convert_to_0_2pi(psi2) - convert_to_0_2pi(psi1)
                bt = np.arctan2(dij[1], dij[0]) 
                br = convert_to_0_2pi(bt) - convert_to_0_2pi(psi1)
                
                VO.append([apex, left_ray, right_ray, dij, vr, br, psir, Xn, Xo, vi, vj]) 
                
            psi = reachable_velocities(vi, VO, wpf, guidance_dict, Xo, Xn)[0]
            
    return psi , collision_flag        


def cost_function1(v, vel_d):
    return np.linalg.norm(np.array(v) - np.array(vel_d))

def colregs_criteria(br, psi_r):
    psi_r = np.mod(psi_r, 2*np.pi)
    br = np.mod(br, 2*np.pi)
    if np.pi/2 < br < 3*np.pi/2 and abs(br-psi_r) < np.pi/2:
        return 'Safe'
    else:
        if 7*np.pi/8 <= psi_r < 9*np.pi/8:
            return 'Head-on'
        elif 9*np.pi/8 <= psi_r < 13*np.pi/8:
            return 'Stand-on in Crossing'
        elif 3*np.pi/8 <= psi_r < 7*np.pi/8:
            return 'Give-way in Crossing'
       
        else:
            psi_rd = (2*np.pi-psi_r) % 2*np.pi
            br_d = (np.pi+br-psi_r) % 2*np.pi
            if 5*np.pi/8 <= br < 11*np.pi/8:
                return 'Overtaking'
            elif 5*np.pi/8 <= br_d < 11*np.pi/8:
                return 'Overtaken'
            elif br_d < np.pi:
                return 'Give-way in Crossing'
            else:
                return 'Stand-on in Crossing'
            

#Reachable Avoidance Velocity (RAV) 
# def reachable_velocities(vel_d, VO, wpf, guidance_dict, Xo, Xn):
#     v = np.array([2,2])
#     alpha = 0.5
#     v_norm = np.linalg.norm(v)
#     # print("v:" ,v_norm)
#     # v_norm1 = np.linalg.norm(vel_d)
#     # print("vel_d:" ,v_norm1)
#     num_points1 = 100
#     num_points2 = 10
#     v_new = []
#     cone_paths = []

#     # Calculate common values
#     d1 = wpf - Xo
#     line1 = 5 * np.array([-d1[1], d1[0]])
#     line2 = 5 * np.array([d1[1], -d1[0]])

#     d1_left = Xo * line1 + d1
#     d1_right = Xo * line2 + d1

#     # Velocity Cone (VO)
#     for vo in VO:
#         apex, left_ray, right_ray, _, _, _, _, _, _, _, _ = vo
#         cone_path_data = [
#             (Path.MOVETO, apex),
#             (Path.LINETO, right_ray),
#             (Path.LINETO, left_ray),
#             (Path.CLOSEPOLY, apex),
#         ]
        
#         cone_path = Path([vertex for code, vertex in cone_path_data], [code for code, _ in cone_path_data])
#         cone_paths.append(cone_path)
        
#     # Define obstacles as rectangles
#     obstacles = [
#         [(Path.MOVETO, (-40, 50)), (Path.LINETO, (-10, 50)), (Path.LINETO, (-10, -50)), (Path.LINETO, (-40, -50)), (Path.CLOSEPOLY, (-40, 50))],
#         [(Path.MOVETO, (30, 50)), (Path.LINETO, (50, 50)), (Path.LINETO, (50, 20)), (Path.LINETO, (30, 20)), (Path.CLOSEPOLY, (30, 50))],
#         [(Path.MOVETO, (30, -20)), (Path.LINETO, (50, -20)), (Path.LINETO, (50, -50)), (Path.LINETO, (30, -50)), (Path.CLOSEPOLY, (30, -20))]
#     ]

#     obstacle_paths = [Path([vertex for code, vertex in obs], [code for code, _ in obs]) for obs in obstacles]
#     # Reachable Velocity (RV)
#     for th in np.linspace(np.pi, -np.pi, num_points1):
#         for mag in np.linspace(0.05, v_norm + 0.05, num_points2):
#             new_vel = [mag * np.cos(th), mag * np.sin(th)]
            
#             v_outside = True
#             for vo, cone_path, ob1 in zip(VO, cone_paths, obstacle_paths):
#                 apex, left_boundary, right_boundary, dij, vr, br, psir, pA, pB, vi, vj = vo 
#                 if cone_path.contains_point(new_vel):                    
#                     v_outside = False
#                     break
#             for ob1 in obstacle_paths:
#                 if ob1.contains_point(new_vel):
#                     v_outside = False
#                     break

#             if v_outside:
#                 v_new.append(new_vel)
    
#     v_new = np.array(v_new)
    
#     # Finding the optimal velocity
#     Vopt_A = min(v_new, key=lambda v: cost_function1(v, vel_d))   
#     psi = np.arctan2(Vopt_A[1], Vopt_A[0])
    
#     for vo, cone_path in zip(VO, cone_paths):
#         apex, left_boundary, right_boundary, dij, vr, br, psir, pA, pB, vi, vj = vo
        
#         cross1 = np.cross(d1_left, pB)
#         cross2 = np.cross(d1_right, pB)
#         case1 = (cross1 > 0 and cross2 < 0) or (cross1 < 0 and cross2 > 0)
#         if case1:
#             psi = np.arctan2(Vopt_A[1], Vopt_A[0])
#         elif np.cross(dij, vj) > 0:
#             psi = np.arctan2(vi[1], vi[0])

#     return psi, v_new


def reachable_velocities(vel_d, VO, wpf, guidance_dict, Xo, Xn):
    v = np.array([2, 2])
    alpha = 0.5
    v_norm = np.linalg.norm(vel_d)
    num_points1 = 100
    num_points2 = 10
    v_new = []
    cone_paths = []

    # Calculate common values
    d1 = wpf - Xo
    line1 = 5 * np.array([-d1[1], d1[0]])
    line2 = 5 * np.array([d1[1], -d1[0]])

    d1_left = Xo * line1 + d1
    d1_right = Xo * line2 + d1

    # Velocity Cone (VO)
    for vo in VO:
        apex, left_ray, right_ray, _, _, _, _, _, _, _, _ = vo
        cone_path_data = [
            (Path.MOVETO, apex),
            (Path.LINETO, right_ray),
            (Path.LINETO, left_ray),
            (Path.CLOSEPOLY, apex),
        ]
        
        cone_path = Path([vertex for code, vertex in cone_path_data], [code for code, _ in cone_path_data])
        cone_paths.append(cone_path)
        
    # Define obstacles as rectangles
    obstacles = [
        [(Path.MOVETO, (-40, 50)), (Path.LINETO, (-10, 50)), (Path.LINETO, (-10, -50)), (Path.LINETO, (-40, -50)), (Path.CLOSEPOLY, (-40, 50))],
        [(Path.MOVETO, (30, 50)), (Path.LINETO, (50, 50)), (Path.LINETO, (50, 20)), (Path.LINETO, (30, 20)), (Path.CLOSEPOLY, (30, 50))],
        [(Path.MOVETO, (30, -20)), (Path.LINETO, (50, -20)), (Path.LINETO, (50, -50)), (Path.LINETO, (30, -50)), (Path.CLOSEPOLY, (30, -20))]
    ]

    obstacle_paths = [Path([vertex for code, vertex in obs], [code for code, _ in obs]) for obs in obstacles]

    # Reachable Velocity (RV)
    for th in np.linspace(np.pi, -np.pi, num_points1):
        for mag in np.linspace(0.05, v_norm + 0.05, num_points2):
            new_vel = [mag * np.cos(th), mag * np.sin(th)]
            
            # Check if new_vel is outside all velocity cones and obstacles
            v_outside_cones = all(not cone_path.contains_point(new_vel) for cone_path in cone_paths)
            v_outside_obstacles = all(not obstacle_path.contains_point(new_vel) for obstacle_path in obstacle_paths)

            if v_outside_cones and v_outside_obstacles:
                v_new.append(new_vel)
    
    v_new = np.array(v_new)
    
    # Finding the optimal velocity
    
    Vopt_A = min(v_new, key=lambda v: cost_function1(v, vel_d))   
    psi = np.arctan2(Vopt_A[1], Vopt_A[0])
    
    for vo, cone_path in zip(VO, cone_paths):
        apex, left_boundary, right_boundary, dij, vr, br, psir, pA, pB, vi, vj = vo
        
        cross1 = np.cross(d1_left, pB)
        cross2 = np.cross(d1_right, pB)
        case1 = (cross1 > 0 and cross2 < 0) or (cross1 < 0 and cross2 > 0)
        if case1:
            psi = np.arctan2(Vopt_A[1], Vopt_A[0])
        elif np.cross(dij, vj) > 0:
            psi = np.arctan2(vi[1], vi[0])

    return psi, v_new
