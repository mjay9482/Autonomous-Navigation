#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 20 16:08:07 2024

@author: mrityunjay
"""

import numpy as np
from guidance_control import ssa

# class Obstacle():
    
#     def __init__(self, env, agent_id, obs_id):
        
#         self.obs_id = obs_id
#         self.u = env.agent_list[obs_id].curr_ss[0] * env.agent_list[obs_id].design_speed / env.agent_list[agent_id].design_speed
#         self.v = env.agent_list[obs_id].curr_ss[1] * env.agent_list[obs_id].design_speed / env.agent_list[agent_id].design_speed
#         self.x = env.agent_list[obs_id].curr_ss[3] * env.agent_list[obs_id].length / env.agent_list[agent_id].length
#         self.y = env.agent_list[obs_id].curr_ss[4] * env.agent_list[obs_id].length / env.agent_list[agent_id].length
#         self.psi = env.agent_list[obs_id].curr_ss[5]
        
#         # agent_id is the index of the agent for which agent with
#         # index id is an obstacle under consideration
#         self.agent_id = agent_id
#         self.agent_u = env.agent_list[agent_id].curr_ss[0]
#         self.agent_v = env.agent_list[agent_id].curr_ss[1]
#         self.agent_x = env.agent_list[agent_id].curr_ss[3]
#         self.agent_y = env.agent_list[agent_id].curr_ss[4]
#         self.agent_psi = env.agent_list[agent_id].curr_ss[5]

#         self.agent_length = env.agent_list[agent_id].length
#         self.agent_speed = env.agent_list[agent_id].design_speed
        
#         self.obs_size = env.agent_list[obs_id].length / env.agent_list[agent_id].length
        
#         x1 = self.agent_x
#         y1 = self.agent_y
#         psi1 = self.agent_psi
        
#         u1 = self.agent_u
#         v1 = self.agent_v
        
#         x2 = self.x
#         y2 = self.y
#         psi2 = self.psi
        
#         u2 = self.u
#         v2 = self.v
        
        
#         u1_glob = u1*np.cos(psi1) - v1*np.sin(psi1)
#         v1_glob = u1*np.sin(psi1) + v1*np.cos(psi1)
        
#         u2_glob = u2*np.cos(psi2) - v2*np.sin(psi2)
#         v2_glob = u2*np.sin(psi2) + v2*np.cos(psi2)
        
#         rel_heading = ssa(psi2 - psi1)
        
#         d = np.sqrt((x1 - x2) **2 + (y1 - y2) ** 2)
        
#         gamma = np.arctan2(y2 - y1, x2 - x1)
        
#         R = np.array([[np.cos(gamma), np.sin(gamma)], [-np.sin(gamma), np.cos(gamma)]])
        
#         rel_vel = (R @ (np.array([u2_glob, v2_glob]) - np.array([u1_glob, v1_glob])))
#         rel_vel_rad = rel_vel[0]
#         rel_vel_tan = rel_vel[1]
        
#         if rel_vel_rad < 0:
#             tcpa = d / np.abs(rel_vel_rad)
#         else:
#             tcpa = np.inf
        
#         if not np.isinf(tcpa):
#             dcpa = tcpa * rel_vel_tan
#         else:
#             dcpa = np.inf
        
#         r_rel = np.array([x1, y1]) - np.array([x2, y2])
#         rhat = r_rel / np.linalg.norm(r_rel)
        
#         u_rel = np.array([u2, v2]) - np.array([u1, v1])
#         uhat = u_rel / np.linalg.norm(u_rel)
        
#         s_th = np.cross(rhat, uhat)
#         c_th = np.dot(rhat, uhat)
#         th = np.arctan2(s_th, c_th)
        
#         self.dist = d
#         self.rel_heading = rel_heading
#         self.gamma = ssa(gamma - psi1)
#         self.rel_vel_rad = rel_vel_rad
#         self.rel_vel_tan = rel_vel_tan
#         self.tcpa = tcpa
#         self.dcpa = dcpa
#         self.theta = th
        
#         self.safe_radius = env.safe_radius



class Obstacle():
    
    def __init__(self, env, agent_id, obs_id):
        self.obs_id = obs_id
        
        # Ensure that design_speed is non-zero before division
        agent_speed = env.agent_list[agent_id].design_speed
        obs_speed = env.agent_list[obs_id].design_speed
        
        if agent_speed != 0:
            self.u = env.agent_list[obs_id].curr_ss[0] * obs_speed / agent_speed
            self.v = env.agent_list[obs_id].curr_ss[1] * obs_speed / agent_speed
        else:
            self.u = 0
            self.v = 0
        
        # Same for agent positions and velocities
        if agent_speed != 0:
            self.x = env.agent_list[obs_id].curr_ss[3] * env.agent_list[obs_id].length / env.agent_list[agent_id].length
            self.y = env.agent_list[obs_id].curr_ss[4] * env.agent_list[obs_id].length / env.agent_list[agent_id].length
        else:
            self.x = 0
            self.y = 0
        
        self.psi = env.agent_list[obs_id].curr_ss[5]
        
        # agent_id is the index of the agent for which agent with
        # index id is an obstacle under consideration
        self.agent_id = agent_id
        self.agent_u = env.agent_list[agent_id].curr_ss[0]
        self.agent_v = env.agent_list[agent_id].curr_ss[1]
        self.agent_x = env.agent_list[agent_id].curr_ss[3]
        self.agent_y = env.agent_list[agent_id].curr_ss[4]
        self.agent_psi = env.agent_list[agent_id].curr_ss[5]

        self.agent_length = env.agent_list[agent_id].length
        self.agent_speed = env.agent_list[agent_id].design_speed
        
        self.obs_size = env.agent_list[obs_id].length / env.agent_list[agent_id].length
        
        x1 = self.agent_x
        y1 = self.agent_y
        psi1 = self.agent_psi
        
        u1 = self.agent_u
        v1 = self.agent_v
        
        x2 = self.x
        y2 = self.y
        psi2 = self.psi
        
        u2 = self.u
        v2 = self.v
        
        
        u1_glob = u1*np.cos(psi1) - v1*np.sin(psi1)
        v1_glob = u1*np.sin(psi1) + v1*np.cos(psi1)
        
        u2_glob = u2*np.cos(psi2) - v2*np.sin(psi2)
        v2_glob = u2*np.sin(psi2) + v2*np.cos(psi2)
        
        rel_heading = ssa(psi2 - psi1)
        
        d = np.sqrt((x1 - x2) **2 + (y1 - y2) ** 2)
        
        gamma = np.arctan2(y2 - y1, x2 - x1)
        
        R = np.array([[np.cos(gamma), np.sin(gamma)], [-np.sin(gamma), np.cos(gamma)]])
        
        rel_vel = (R @ (np.array([u2_glob, v2_glob]) - np.array([u1_glob, v1_glob])))
        rel_vel_rad = rel_vel[0]
        rel_vel_tan = rel_vel[1]
        
        if rel_vel_rad < 0:
            tcpa = d / np.abs(rel_vel_rad)
        else:
            tcpa = np.inf
        
        if not np.isinf(tcpa):
            dcpa = tcpa * rel_vel_tan
        else:
            dcpa = np.inf
        
        r_rel = np.array([x1, y1]) - np.array([x2, y2])
        rhat = r_rel / np.linalg.norm(r_rel) if np.linalg.norm(r_rel) != 0 else r_rel
        
        u_rel = np.array([u2, v2]) - np.array([u1, v1])
        
        # Avoid division by zero when normalizing u_rel
        norm_u_rel = np.linalg.norm(u_rel)
        uhat = u_rel / norm_u_rel if norm_u_rel != 0 else u_rel
        
        s_th = np.cross(rhat, uhat)
        c_th = np.dot(rhat, uhat)
        th = np.arctan2(s_th, c_th)
        
        self.dist = d
        self.rel_heading = rel_heading
        self.gamma = ssa(gamma - psi1)
        self.rel_vel_rad = rel_vel_rad
        self.rel_vel_tan = rel_vel_tan
        self.tcpa = tcpa
        self.dcpa = dcpa
        self.theta = th
        
        self.safe_radius = env.safe_radius
