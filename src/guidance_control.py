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
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import LinearRing

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

def control(psid, vd,  ss, uerr_int, obstacles, control_dict=None):   
    # n_c = 115.5 / 60  # commanded propeller speed (1:75.5 scaled KCS to achieve 1.42 m/s in steady state)
    psi = ss[5]
    r = ss[2]
    
    u = ss[0]
    v = ss[1]
    s_err = ss[9]
    s = np.sqrt(u **2 + v **2)
    n_c = -3 * (s - 1) - 0.1 * s_err       #for VO 
    # n_c = -10 * (s - vd) - 1 * s_err
    
    # with_speed_control = False            # True for Speed control
    # if with_speed_control:
    #     if obstacles:
    #         Kp_P = 100
    #         Ki_P = 10
    #         n_c = Kp_P * (vd - s) + Ki_P * uerr_int
    #         # n_c = saturate(n_c, 800.0/75.5)
    #     else:
    #         n_c = -3 * (s - 1) - 0.1 * s_err
    # else:
    #     n_c = -3 * (s - 1) - 0.1 * s_err
    
    # Kp_P = 100
    # Ki_P = 10
    # n_c = Kp_P * (vd - s) + Ki_P * uerr_int
    # n_c = saturate(n_c, 800.0/75.5)
    
    if control_dict['control_type'] == 'PD':
        
        Kp = control_dict['Kp']
        Kd = control_dict['Kd']
        
        e = ssa(psi - psid)                             
        delta_c = -Kp * e - Kd * r                      
        delta_c = saturate(delta_c, 35*np.pi/180)   
    return delta_c, n_c

def ilos_guidance(ss, gwp=None, iwp=None, dLA=1, ki=0, kappa=0):
    x_own = ss[3]
    y_own = ss[4]
    psi = ss[5]
    yp_int = ss[8]      # For implementing ILOS
    
    xg = gwp[0]         # Goal waypoint
    yg = gwp[1]
    
    # xi = ss[3]
    # yi = ss[4]
    xi = iwp[0]         # Initial waypoint
    yi = iwp[1]
    
    pi_p = np.arctan2(yg - yi, xg - xi)

    # For calculating yp_e
    R_np = np.array([[np.cos(pi_p), -np.sin(pi_p)], 
                     [np.sin(pi_p), np.cos(pi_p)]])
    
    # err (2*1 vector) contains xpe and ype
    err = np.transpose(R_np) @ np.array([[x_own - xi], [y_own - yi]])
    goal = np.transpose(R_np) @ np.array([[xg - xi], [yg - yi]])

    xpe, ype = err           
    xge, yge = goal
    
    kp = 1/dLA
    # psi_des = pi_p - np.arctan(kp*ype + ki*yp_int)
    if xpe < xge:
        psi_des = pi_p - np.arctan(kp*ype + ki*yp_int)             
    else:
        psi_des = pi_p - np.pi + np.arctan(kp*ype + ki*yp_int)      
    
    ypd_int = (dLA*ype)/(dLA**2 + (ype + kappa*yp_int)**2)      

    v_des = ss[:2]
    return psi_des, v_des, ypd_int, xpe, ype

def cost_function1(v, vel_d):
    return np.linalg.norm(np.array(v) - np.array(vel_d))
    
# Mapping Configuration Space to Velocity Space
def reactive_guidance(wpf, design_speed, obstacles, guidance_dict, boundary_grad=None):
    grad = np.zeros(2)
    collision_flag = 0
    collision_tol = guidance_dict['collision_tolerance']
    if not obstacles and np.linalg.norm(boundary_grad) < 1e-2:
        return None, collision_flag
    if obstacles:
        Xg = np.array(wpf)
        VO = []
        guidance_type = guidance_dict['reactive_guidance']
        Xn = np.array([obstacles[0].agent_x, obstacles[0].agent_y])

        for obs in obstacles:
            Xo = np.array([obs.x, obs.y])

            # Check for collision with obstacle
            if np.linalg.norm(Xn - Xo) < collision_tol:
                collision_flag = 1
    
            R_tol = guidance_dict['collision_tolerance']

            dij = Xo - Xn
            dij_norm = np.linalg.norm(dij)
            dij_p = dij / dij_norm

            # Agent and obstacle velocities
            u1_glob, v1_glob = global_velocity(obs.agent_u, obs.agent_v, obs.agent_psi)
            u2_glob, v2_glob = global_velocity(obs.u, obs.v, obs.psi)

            vi = np.array([u1_glob, v1_glob])
            vj = np.array([u2_glob, v2_glob])

            # Construct Velocity Obstacle (VO or RVO)
            if guidance_type == 'vo':
                apex = vj
            elif guidance_type == 'rvo':
                # apex = (vj + vi)/2
                apex = vj

            left_ray, right_ray = compute_velocity_cone(apex, dij, dij_p, dij_norm, R_tol)

            VO.append([apex, left_ray, right_ray, obs.agent_psi, dij, Xn, Xo, vi, vj])
            
        grad = reachable_avoidance_velocities(vi, vj, design_speed, obs.agent_psi, VO, wpf, guidance_dict, Xn, Xo)
    
    # if boundary_grad is not None and np.linalg.norm(boundary_grad) > 1e-3:
    #     grad += boundary_grad/5
    psi = np.arctan2(grad[1],grad[0])
    return psi, grad, collision_flag

# Local velocities to Global Velocities
def global_velocity(u, v, psi):
    u_glob = u * np.cos(psi) - v * np.sin(psi)
    v_glob = u * np.sin(psi) + v * np.cos(psi)
    return u_glob, v_glob

# For Computation of velocity cone
# def compute_velocity_cone(apex, dij, dij_p, dij_norm, R_tol):
#     x1 = apex + dij + (2 * R_tol) * np.array([-dij_p[1], dij_p[0]])
#     phi1 = np.arctan2(x1[1], x1[0])
#     left_ray = apex + dij_norm * np.array([np.cos(phi1), np.sin(phi1)])

#     x2 = apex + dij + (2 * R_tol) * np.array([dij_p[1], -dij_p[0]])
#     phi2 = np.arctan2(x2[1], x2[0])
#     right_ray = apex + dij_norm * np.array([np.cos(phi2), np.sin(phi2)])

#     return left_ray, right_ray

def compute_velocity_cone(apex, dij, dij_p, dij_norm, R_tol):
    if dij_norm < R_tol:
        dij_norm = 2 * R_tol
    vector_angle = np.arctan2(dij[1], dij[0])
    cone_half_angle = np.arctan2(R_tol, dij_norm)  
    left_angle = vector_angle + 2*cone_half_angle
    right_angle = vector_angle - 2*cone_half_angle
    left_ray = apex + 3*dij_norm * np.array([np.cos(left_angle), np.sin(left_angle)])
    right_ray = apex + 3*dij_norm * np.array([np.cos(right_angle), np.sin(right_angle)])
    cross_product = np.cross(dij, left_ray - apex)
    if cross_product < 0:  
        left_ray, right_ray = right_ray, left_ray
    return left_ray, right_ray

# To define the conical region of the obstacle for dynamic obstacle
def velocity_cone(apex, left_ray, right_ray):
    vertices = [apex , right_ray, left_ray, apex]
    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    return Path(vertices, codes)

# Reachable Avoidance Velocity (RAV) 
def reachable_avoidance_velocities(vi, vj, design_speed, psi, VO, wpf, guidance_dict, pA, pB):
    # v_norm = np.linalg.norm(vi)
    # design_speed = 1.
    v_norm = design_speed/12
    vel_d =  ((wpf - pA) / np.linalg.norm(wpf - pA)) * v_norm
    num_points1 = 80
    num_points2 = 80
    v_new = []
    cone_paths = []
    guidance_type = guidance_dict['reactive_guidance']
    collision_radius =  2 * guidance_dict['collision_tolerance']
    
    for vo in VO:
        apex, left_ray, right_ray, _, _, _, _, _, _ = vo
        cone_paths.append(velocity_cone(apex, left_ray, right_ray))
    
    for vo in VO:
        psi , vi , vj = vo[3], vo[7], vo[8]
        for th in np.linspace(psi, psi + 2*np.pi, num_points1):
            for mag in np.linspace(v_norm/5, v_norm + 4 , num_points2):
                new_vel = [mag * np.cos(th), mag * np.sin(th)]
                if guidance_type == 'vo':
                    new_vel_chk = [mag * np.cos(th), mag * np.sin(th)]
                else:
                    new_vel_chk = [2*mag * np.cos(th) - vi[0], 2*mag * np.sin(th) - vi[1]]
                for cone_path in cone_paths:
                    if not cone_path.contains_point(new_vel_chk):
                        v_new.append(new_vel)
 
    v_new = np.array(v_new)
    Vopt_A = min(v_new, key=lambda v: cost_function1(v, vel_d))
    psi = np.arctan2(Vopt_A[1], Vopt_A[0])
    return Vopt_A
