#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 20 18:27:52 2024

@author: mrityunjay
"""

from env import Environment
import logger as log
from plotting import generate_plots, generate_mat


def execute(params, gif_id=None, plot_flag=False, mat_flag=False, stat_id=None, mat_save_flag=True):
                                                             
    log.initiate_log()
    log.info(f'Starting {params.case_dir}')
    
    env = Environment(params)
    env.reset()
    
    for j in range(env.n_timesteps - 1):
        env.step()
        for i in range(len(env.agent_list)):
            ss = env.agent_list[i].curr_ss
            # if True:
            #     log.info(f'Timestep: {j}, Agent ID: {env.agent_list[i].id} --> u: {ss[0]:.4f}, v: {ss[1]:.4f}, r: {ss[2]:.4f}, x: {ss[3]:.4f}, y: {ss[4]:.4f}, psi: {ss[5]:.4f}, delta: {ss[6]:.4f}, n_prop: {ss[7]:.4f}, yp_int: {ss[8]:.4f}')
    
    if plot_flag or mat_flag:
        env_data = generate_plots(env, gif_id=gif_id, mat_flag=mat_flag, plot_flag=plot_flag, stat_id=stat_id, mat_save_flag=mat_save_flag)
    
    log.info(f'Finished {params.case_dir}')
    return env_data








