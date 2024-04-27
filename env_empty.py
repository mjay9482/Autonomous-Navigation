#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 15:33:49 2024

@author: mrityunjay
"""
import numpy as np


class Params_empty():
    
    def __init__(self, str1, seed):
        self.case_dir = str1
        self.seed = seed

class Agent_empty():
    
    def __init__(self, agent_dict):
        
        self.id = agent_dict['id'][0, 0][0, 0]
        self.agent_type = agent_dict['agent_type'][0, 0][0]
        self.length = agent_dict['length'][0, 0][0, 0]
        self.design_speed = agent_dict['design_speed'][0, 0][0, 0]
        self.Fn = agent_dict['Fn'][0, 0][0, 0]
        
        self.state = agent_dict['termination_state'][0, 0][0, 0]
        self.t = np.array(agent_dict['time'][0, 0][0])
        self.termination_timestep_indx = agent_dict['termination_timestep_indx'][0, 0][0, 0]
        self.ss = agent_dict['state_space'][0, 0]
        self.ype = agent_dict['ype'][0, 0][0]
        self.xpe = agent_dict['xpe'][0, 0][0]
        self.psi_des = agent_dict['psi_des'][0, 0][0]
        self.delta_c = agent_dict['delta_c'][0, 0][0]
        self.n_c = agent_dict['n_c'][0, 0][0]
        
        self.mcte = agent_dict['mcte']
        self.control_effort = agent_dict['control_effort']
        self.apf_compute_time = agent_dict['vo_compute_time']
        self.apf_compute_calls = agent_dict['vo_compute_calls']
        self.collision_state = agent_dict['collision_state']
        
        self.curr_ss = self.ss[-1, :]
        
        
        if len(agent_dict['guidance_dict'][0, 0]) == 0:
            
            self.guidance_dict = None
            
        else:
            
            guidance_dict_temp = agent_dict['guidance_dict'][0, 0][0, 0]
            self.guidance_dict = {
                'waypoints': guidance_dict_temp[0].tolist(), 
                'dLA' : guidance_dict_temp[1][0, 0],
                'ki' : guidance_dict_temp[2][0, 0],
                'kappa' : guidance_dict_temp[3][0, 0],
                'tracking_tolerance' : guidance_dict_temp[4][0, 0],
                'collision_tolerance' : guidance_dict_temp[5][0, 0],
                'reactive_guidance' : guidance_dict_temp[6][0],
            }
        
        if len(agent_dict['control_dict'][0, 0]) == 0:
            
            self.control_dict = None
            
        else:
            
            control_dict_temp = agent_dict['control_dict'][0, 0][0, 0]
            
            self.control_dict = {
                'control_type': control_dict_temp[0][0], 
                'Kp': control_dict_temp[1][0, 0], 
                'Kd': control_dict_temp[2][0, 0]
            }


class Environment_empty():
    
    def __init__(self, mdict):
        
        self.case_name = mdict['case_name'][0]
        self.seed = mdict['seed'][0, 0]
        self.params = Params_empty(self.case_name, self.seed)
        
        self.inter_agent_dist = mdict['inter_agent_dist']
        self.inter_agent_heading = mdict['inter_agent_heading']
        self.inter_agent_vector_dir = mdict['inter_agent_vector_dir']
        self.inter_agent_rel_vel_rad = mdict['inter_agent_rel_vel_rad']
        self.inter_agent_rel_vel_tan = mdict['inter_agent_rel_vel_tan']
        self.inter_agent_tcpa = mdict['inter_agent_tcpa']
        self.inter_agent_dcpa = mdict['inter_agent_dcpa']
        self.inter_agent_vector_dir = mdict['inter_agent_vector_dir']
        self.inter_agent_theta = mdict['inter_agent_theta']
        self.safe_radius = mdict['safe_radius'][0, 0]
        self.nagent = mdict['nagent'][0, 0]
        self.t = mdict['t'][0]
        self.n_timesteps = len(self.t)
        
        agent_list = []
        
        for i, agent_dict in enumerate(mdict['agents'][0]):
            
            agent = Agent_empty(agent_dict)
            
            agent_list.append(agent)
        
        self.agent_list = agent_list
