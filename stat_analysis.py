#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 28 12:59:02 2024

@author: mrityunjay
"""

import numpy as np
from scipy.io import loadmat
import os
import matplotlib.pyplot as plt
from pprint import pprint
from plotting import generate_path_plot, generate_trajectory_anim
from env_empty import Environment_empty

mat_dir = os.path.join(os.getcwd(), f'../simdata_stat')

mcte_all_vo = []
control_effort_all_vo = []
vo_compute_time_all_vo = []
time_to_goal_all_vo = []
success_all_vo = []

vo_list = ['vo', 'rvo']

for vo in vo_list:
    
    mcte_one_vo = []
    control_effort_one_vo = []
    vo_compute_time_one_vo = []
    time_to_goal_one_vo = []
    success_one_vo = []
    
    for i in range(5):
        
        mdict = loadmat(f'{mat_dir}/simdata_stat_{i+1:02d}_{vo}.mat')
        
        mcte = mdict['mcte']
        control_effort = mdict['control_effort']
        vo_compute_time = mdict['vo_compute_time']
        time_to_goal = mdict['time_to_goal']
        success = mdict['success']
        
        mcte_one_vo.append(mcte.flatten().tolist())
        control_effort_one_vo.append(control_effort.flatten().tolist())
        vo_compute_time_one_vo.append(vo_compute_time.flatten().tolist())
        time_to_goal_one_vo.append(time_to_goal.flatten().tolist())
        success_one_vo.append(success.flatten().tolist())
    
    mcte_all_vo.append(mcte_one_vo)
    control_effort_all_vo.append(control_effort_one_vo)
    vo_compute_time_all_vo.append(vo_compute_time_one_vo)
    time_to_goal_all_vo.append(time_to_goal_one_vo)
    success_all_vo.append(success_one_vo)
    

mcte_all_vo = np.array(mcte_all_vo)
control_effort_all_vo = np.array(control_effort_all_vo)
vo_compute_time_all_vo = np.array(vo_compute_time_all_vo)
time_to_goal_all_vo = np.array(time_to_goal_all_vo)
success_all_vo = np.array(success_all_vo)

color_ord = ['k', 'r--']

fig, ax = plt.subplots()

for i in range(len(vo_list)):
    y = np.mean(mcte_all_vo, axis=2)[i, :]
    ci = 1.96 * np.std(mcte_all_vo, axis=2)[i, :] / np.sqrt(1000)
    ax.errorbar(np.arange(5), y, yerr=ci, fmt=color_ord[i], label=vo_list[i], capsize=3, capthick=1, marker='o')
    # ax.fill_between(np.arange(5), (y-ci), (y+ci), color=color_ord[i][0], alpha=.2)
    
ax.set_title(f'Mean MCTE',fontsize=15)
ax.set_ylabel("MCTE",fontsize=15)
ax.set_xlabel("static+dynamic",fontsize=15)
ax.set_xticks([0,1,2,3,4])
ax.set_xticklabels(['1+2', '2+3', '2+5', '3+7', '4+9'])
ax.tick_params(axis='x', labelsize=15)
ax.tick_params(axis='y', labelsize=15)
ax.legend(loc="best")
ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
plt.savefig('mcte_mean.png', bbox_inches='tight')
plt.savefig('mcte_mean.svg', bbox_inches='tight')
plt.cla()

for i in range(len(vo_list)):
    y = np.mean(control_effort_all_vo, axis=2)[i, :]
    ci = 1.96 * np.std(control_effort_all_vo, axis=2)[i, :] / np.sqrt(1000)
    ax.errorbar(np.arange(5), y, yerr=ci, fmt=color_ord[i], label=vo_list[i], capsize=3, capthick=1, marker='o')
    
ax.set_title(f'Mean Control Effort',fontsize=15)
ax.set_ylabel("CE",fontsize=15)
ax.set_xlabel("static+dynamic",fontsize=15)
ax.set_xticks([0,1,2,3,4])
ax.set_xticklabels(['1+2', '2+3', '2+5', '3+7', '4+9'])
ax.tick_params(axis='x', labelsize=15)
ax.tick_params(axis='y', labelsize=15)
ax.legend(loc="best")
ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
plt.savefig('control_effort_mean.png', bbox_inches='tight')
plt.savefig('control_effort_mean.svg', bbox_inches='tight')
plt.cla()

for i in range(len(vo_list)):
    y = np.mean(vo_compute_time_all_vo, axis=2)[i, :]
    ci = 1.96 * np.std(vo_compute_time_all_vo, axis=2)[i, :] / np.sqrt(1000)
    ax.errorbar(np.arange(5), y*1.e6, yerr=ci*1.e6, fmt=color_ord[i], label=vo_list[i], capsize=3, capthick=1, marker='o')
    # ax.plot(np.arange(5), np.mean(vo_compute_time_all_vo, axis=2)[i, :] * 1.e6, color_ord[i], label=vo_list[i])
    
ax.set_title(f'Mean computation time per vo call',fontsize=15)
ax.set_ylabel("Computation time ($\mu$s)",fontsize=15)
ax.set_xlabel("static+dynamic",fontsize=15)
ax.set_xticks([0,1,2,3,4])
ax.set_xticklabels(['1+2', '2+3', '2+5', '3+7', '4+9'])
ax.tick_params(axis='x', labelsize=15)
ax.tick_params(axis='y', labelsize=15)
ax.legend(loc="best")
ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
plt.savefig('vo_compute_time_mean.png', bbox_inches='tight')
plt.savefig('vo_compute_time_mean.svg', bbox_inches='tight')
plt.cla()

for i in range(len(vo_list)):
    y = np.mean(np.ma.masked_invalid(time_to_goal_all_vo), axis=2)[i, :]
    ci = 1.96 * np.std(np.ma.masked_invalid(time_to_goal_all_vo), axis=2)[i, :] / np.sqrt(1000)
    ax.errorbar(np.arange(5), y, yerr=ci, fmt=color_ord[i], label=vo_list[i], capsize=3, capthick=1, marker='o')
    # ax.plot(np.arange(5), np.mean(np.ma.masked_invalid(time_to_goal_all_vo), axis=2)[i, :], color_ord[i], label=vo_list[i])
    
ax.set_title(f'Mean time to reach goal',fontsize=15)
ax.set_ylabel("Nondimensional time",fontsize=15)
ax.set_xlabel("static+dynamic",fontsize=15)
ax.set_xticks([0,1,2,3,4])
ax.set_xticklabels(['1+2', '2+3', '2+5', '3+7', '4+9'])
ax.tick_params(axis='x', labelsize=15)
ax.tick_params(axis='y', labelsize=15)
ax.legend(loc="best")
ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
plt.savefig('time_to_goal_mean.png', bbox_inches='tight')
plt.savefig('time_to_goal_mean.svg', bbox_inches='tight')
plt.cla()

for i in range(len(vo_list)):
    y = np.mean(success_all_vo, axis=2)[i, :]
    ci = 1.96 * np.std(success_all_vo, axis=2)[i, :] / np.sqrt(1000)
    ax.errorbar(np.arange(5), y*100, yerr=ci*100, fmt=color_ord[i], label=vo_list[i], capsize=3, capthick=1, marker='o')
    # ax.plot(np.arange(5), np.mean(success_all_vo, axis=2)[i, :] * 100, color_ord[i], label=vo_list[i])
    
ax.set_title(f'Mean Success Rate',fontsize=15)
ax.set_ylabel("Success rate in %",fontsize=15)
ax.set_xlabel("static+dynamic",fontsize=15)
ax.set_xticks([0,1,2,3,4])
ax.set_xticklabels(['1+2', '2+3', '2+5', '3+7', '4+9'])
ax.tick_params(axis='x', labelsize=15)
ax.tick_params(axis='y', labelsize=15)
ax.legend(loc="best")
ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
plt.savefig('success_mean.png', bbox_inches='tight')
plt.savefig('success_mean.svg', bbox_inches='tight')
plt.cla()



mdict = loadmat(f'{mat_dir}/simdata_stat_01_mvortex.mat')

k = 0
for i, env_dict in enumerate(mdict['env_data_list'].flatten().tolist()):
    
    if i == 4: #env_dict[0, 0]['agents'][0, 0]['collision_state'][0, 0][0, 0] == 1:
        print(env_dict[0, 0]['agents'][0, 0]['id'][0, 0][0, 0])
        env = Environment_empty(env_dict[0, 0])
        generate_path_plot(env, f'{mat_dir}/stat_01')
        generate_trajectory_anim(env, f'{mat_dir}/stat_01', None)
        exit()
        
        
    