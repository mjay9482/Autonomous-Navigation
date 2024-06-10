#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 22 17:04:06 2024

@author: mrityunjay
"""

import numpy as np
import os
from scipy.io import loadmat, savemat, whosmat
from env_empty import Environment_empty
from load_simdata import load_simdata_stat, load_simdata_indv
import run
from IPython import display

def load_env(path_to_mat, stat_id):
    mdict = loadmat(path_to_mat)
    env = Environment_empty(mdict['env_data_list'][0, stat_id][0, 0])
    
    import cases_input.case00 as params
    params.case_dir = env.case_name
    params.time_step = env.t[1] - env.t[0]
    params.Tmax = env.t[-1]
    params.safe_radius = 15
    
    params.agent_list = []
    params.agent_size = []
    params.agent_froude = []
    params.agent_init_state = []
    params.agent_plant = []
    params.agent_controller = []
    params.agent_guidance = []
    
    agent_list = np.arange(len(env.agent_list)).tolist()
    # agent_list = [0, 3, 4, 5, 8, 9, 10, 11, 12, 13]
    
    for j in agent_list:
        agent = env.agent_list[j]
        params.agent_list.append(agent.agent_type)
        params.agent_size.append(agent.length)
        params.agent_froude.append(agent.Fn)
        params.agent_init_state.append(agent.ss[0, :].tolist())
        params.agent_controller.append(agent.control_dict)
        params.agent_guidance.append(agent.guidance_dict)
        
        if agent.agent_type == 'kcs':
            params.agent_plant.append(params.kcs_params)
            params.agent_guidance[-1]['reactive_guidance'] = 'vo'
        if agent.agent_type == 'kinematic':
            params.agent_plant.append(None)            
    
    return env, params

path_to_mat = os.path.join(os.getcwd())#, f'../simdata_stat/benchmark/test_03/simdata_stat_05_mvortex.mat')

stat_id = 299
env, params = load_env(path_to_mat, stat_id)

_ = run.execute(params, gif_id=None, plot_flag=True, mat_flag=True, stat_id=stat_id, mat_save_flag=True)
