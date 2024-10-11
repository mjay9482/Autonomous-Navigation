#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 12:37:41 2024

@author: mrityunjay
"""

import matplotlib.pyplot as plt
import os
import glob
from pathlib import Path
from matplotlib.patches import Circle, Rectangle
from matplotlib.animation import FuncAnimation
import numpy as np
from guidance_control import ssa_array
from scipy.io import savemat, loadmat
import logger as log

def generate_plots(env, gif_id=None, mat_flag=True, plot_flag=True, stat_id=None, mat_save_flag=True):    
    
    if plot_flag or mat_save_flag:
        if stat_id is None:
            case_dir = os.path.join(os.getcwd(), '../plots/' , env.params.case_dir)
        else:
            case_dir = os.path.join(os.getcwd(), '../plots/' , env.params.case_dir, f'./stat_{stat_id:02d}')

        
        if os.path.exists(case_dir):
            if stat_id is None:
                files = glob.glob(f'{case_dir}/*')
            else:
                files = glob.glob(f'{case_dir}/*')
            
            for f in files:
                os.remove(f)
        else:            
            Path(case_dir).mkdir(parents=True, exist_ok=True)
    else:
        case_dir = None
    
    if plot_flag:
        log.info(f'Generating plots for {env.params.case_dir}')
        for agent in env.agent_list:
            generate_agent_plots(agent, case_dir)
        
        generate_env_plots(env, case_dir, gif_id=gif_id)
    
    mdict = None
    if mat_flag:
        log.info(f'Generating data in MAT format for {env.params.case_dir}')
        mdict = generate_mat(env, case_dir, save_flag=mat_save_flag)    
    return mdict

def generate_mat(env, case_dir, save_flag=True):
    
    mdict = {}
    mdict['case_name'] = env.params.case_dir
    #mdict['seed'] = env.params.seed
    mdict['inter_agent_dist'] = env.inter_agent_dist
    mdict['inter_agent_heading'] = env.inter_agent_heading
    mdict['inter_agent_vector_dir'] = env.inter_agent_vector_dir
    mdict['inter_agent_rel_vel_rad'] = env.inter_agent_rel_vel_rad
    mdict['inter_agent_rel_vel_tan'] = env.inter_agent_rel_vel_tan
    mdict['inter_agent_tcpa'] = env.inter_agent_tcpa
    mdict['inter_agent_dcpa'] = env.inter_agent_dcpa
    mdict['inter_agent_vector_dir'] = env.inter_agent_vector_dir
    mdict['inter_agent_theta'] = env.inter_agent_theta
    mdict['safe_radius'] = env.safe_radius
    mdict['nagent'] = len(env.agent_list)
    mdict['t'] = env.t
    
    agents_dict = []
    
    for i, agent in enumerate(env.agent_list):
        
        agent_dict = {}
        
        agent_dict['id'] = agent.id
        agent_dict['agent_type'] = agent.agent_type
        agent_dict['length'] = agent.length
        agent_dict['design_speed'] = agent.design_speed
        agent_dict['Fn'] = agent.Fn
        
        agent_dict['termination_state'] = agent.state
        agent_dict['time'] = agent.t
        agent_dict['termination_timestep_indx'] = agent.termination_timestep_indx
        agent_dict['state_space'] = agent.ss
        agent_dict['ype'] = agent.ype
        agent_dict['xpe'] = agent.xpe
        agent_dict['psi_des'] = agent.psi_des
        agent_dict['delta_c'] = agent.delta_c
        agent_dict['n_c'] = agent.n_c
        agent_dict['mcte'] = agent.mcte
        agent_dict['control_effort'] = agent.control_effort
        agent_dict['vo_compute_time'] = agent.vo_compute_time
        agent_dict['vo_compute_calls'] = agent.vo_compute_calls
        agent_dict['collision_state'] = agent.collision_state
        
        if agent.guidance_dict is not None:
            agent_dict['guidance_dict'] = agent.guidance_dict
        else:
            agent_dict['guidance_dict'] = []
        
        if agent.control_dict is not None:
            agent_dict['control_dict'] = agent.control_dict
        else:
            agent_dict['control_dict'] = []
        
        agents_dict.append(agent_dict)
    
    mdict['agents'] = agents_dict
    
    if save_flag:
        savemat(f'{case_dir}/simdata.mat', mdict)
    
    return mdict

def generate_env_plots(env, case_dir, gif_id):
    
    generate_path_plot(env, case_dir)
    
    generate_obs_dist_plot(env, case_dir)
    
    generate_obs_rel_vel_plot(env, case_dir)
    
    generate_obs_rel_heading_plot(env, case_dir)
    
    generate_obs_tcpa_dcpa_plot(env, case_dir)
    
    generate_trajectory_anim(env, case_dir, gif_id)

def generate_agent_plots(agent, case_dir):
    
    plot_path(agent, case_dir)
    
    plot_cross_track_error(agent, case_dir)
    
    plot_heading(agent, case_dir)
    
    plot_delta(agent, case_dir)
    
    plot_state_space(agent, case_dir)
    # pass

def generate_obs_tcpa_dcpa_plot(env, case_dir):
    
    for i in range(len(env.agent_list)):
        
        agent = env.agent_list[i]
    
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i != j:
                temp = np.log(env.inter_agent_tcpa[i, j, :])
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'TCPA between obstacle and Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Log of TCPA",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        # ax.legnd(loc='best')
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_tcpa.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_tcpa.svg', format='svg', bbox_inches='tight')        
        plt.close()
        
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                temp = np.log(env.inter_agent_dcpa[i, j, :])
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'DCPA between obstacle and Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Log of DCPA",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_dcpa.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_dcpa.svg', format='svg', bbox_inches='tight')
        plt.close()

def generate_obs_rel_heading_plot(env, case_dir):
    
    for i in range(len(env.agent_list)):
        
        agent = env.agent_list[i]
    
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                temp = env.inter_agent_heading[i, j, :] * 180 / np.pi
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'Relative heading between obstacle and Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Relative heading in deg",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_rel_heading.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_rel_heading.svg', format='svg', bbox_inches='tight')
        plt.close()
        
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                temp = ssa_array(env.inter_agent_vector_dir[i, j, :]) * 180 / np.pi
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'Relative bearing of obstacle from Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Relative bearing in deg",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_bearing.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_bearing.svg', format='svg', bbox_inches='tight')
        plt.close()
        
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                temp = ssa_array(env.inter_agent_theta[i, j, :]) * 180 / np.pi
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'Encounter angle between relative velocity and\n line joining obstacle and Agent {agent.id}',fontsize=15)
        ax.set_ylabel('$\\theta$ in deg',fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_theta.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_theta.svg', format='svg', bbox_inches='tight')
        plt.close()


def generate_obs_rel_vel_plot(env, case_dir):
    
    for i in range(len(env.agent_list)):
        
        agent = env.agent_list[i]
    
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                temp = env.inter_agent_rel_vel_rad[i, j, :]
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'Relative radial velocity between obstacle and Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Non-dimensional relative radial velocity",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_rel_vel_rad.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_rel_vel_rad.svg', format='svg', bbox_inches='tight')
        plt.close()
        
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                temp = env.inter_agent_rel_vel_tan[i, j, :]
                dist = env.inter_agent_dist[i, j, :]
                temp[dist > env.safe_radius] = np.nan
                ax.plot(env.t, temp, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'Relative tangential velocity between obstacle and Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Non-dimensional relative tangential velocity",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_rel_vel_tan.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_rel_vel_tan.svg', format='svg', bbox_inches='tight')
        plt.close()
    

def generate_obs_dist_plot(env, case_dir):
    
    for i in range(len(env.agent_list)):
        
        agent = env.agent_list[i]
    
        fig, ax = plt.subplots()
        
        for j in range(len(env.agent_list)):
            if i!= j:
                dist = env.inter_agent_dist[i, j, :]
                dist[dist > env.safe_radius] = np.nan
                ax.plot(env.t, dist, linewidth=1.5, label=f'Agent {env.agent_list[j].id}')
        
        if agent.guidance_dict is not None:
            ax.plot(np.array([env.t[0], env.t[-1]]), agent.guidance_dict['collision_tolerance']*np.array([1, 1]), '--k', label='Collision Threshold')
        
        # ax.set_xlim([env.t[0], env.t[-1]])
        ax.set_title(f'Distance from obstacles of Agent {agent.id}',fontsize=15)
        ax.set_ylabel("Distance in ship lengths",fontsize=15)
        ax.set_xlabel("$t'$",fontsize=15)
        ax.tick_params(axis='x', labelsize=15)
        ax.tick_params(axis='y', labelsize=15)
        # ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        ax.legend(loc='best')
        ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_dist.pdf', format='pdf', bbox_inches='tight')
        plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_{agent.id:02d}_obs_dist.svg', format='svg', bbox_inches='tight')
        plt.close()
    
    
def generate_path_plot(env, case_dir):
    
    x_ship = np.array([-0.5, -0.5, 0.25, 0.5, 0.25, -0.5, -0.5, 0.5, 0.25, 0, 0])
    y_ship = 16.1 / 230 * np.array([-1, 1, 1, 0, -1, -1, 0, 0, 1, 1, -1])
    
    fig, ax = plt.subplots()
    ax.invert_yaxis()
    ax.set_aspect('equal', 'box')
    #print(len(env.agent_list))
    agent0 = env.agent_list[0]
    
    for agent in env.agent_list:
        
        if agent.design_speed != 0:
            
            x = agent.ss[:, 3] * agent.length / agent0.length
            y = agent.ss[:, 4] * agent.length / agent0.length
            psi = agent.ss[:, 5]
            
            ax.plot(x, y, label=f'Agent {agent.id}')
            
            if agent.guidance_dict is not None:
                wp = np.array(agent.guidance_dict['waypoints'])
                x_wp = wp[:, 0] * agent.length / agent0.length
                y_wp = wp[:, 1] * agent.length / agent0.length
                
                ax.plot(x_wp, y_wp, '*', markersize=12,label=f'Agent {agent.id} Waypoints')
                for i in range(1, len(x_wp)):
                    # ax.annotate(f'WP {i+1}', xy=(x_wp[i], y_wp[i]), xytext=(x_wp[i] + 4, y_wp[i] + 4))
                    
                    circle = Circle((x_wp[i], y_wp[i]), 
                        agent.guidance_dict['tracking_tolerance'] * agent.length / agent0.length, 
                        facecolor='none', edgecolor='k', linestyle = 'dotted', linewidth = 1.5)
                    
                    ax.add_patch(circle)
                    
        else:
            
            # ax.plot(agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length, 
            #     '*k', markersize=12, label=f'Agent {agent.id} - Obstacle')
            
            # circle = Circle((agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length), 
            #     env.safe_radius, 
            #     facecolor='none', edgecolor='k', linestyle='dotted', linewidth=1.5)
            
            # ax.add_patch(circle)
            
            rectangle = Rectangle((agent.curr_ss[3] * agent.length / agent0.length - env.safe_radius/2, agent.curr_ss[4] * agent.length / agent0.length - env.safe_radius/2), 
                                  5*env.safe_radius, 5*env.safe_radius, facecolor = 'none', edgecolor = 'k', linestyle = 'dotted', linewidth = 1.5)
            ax.add_patch(rectangle)
            
            # circle = Circle((agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length), 
            #     agent.length / (2 * agent0.length), 
            #     facecolor='k', edgecolor='k', linewidth=1.5)
            
            # ax.add_patch(circle)
            
            rectangle = Rectangle((agent.curr_ss[3] * agent.length / agent0.length - env.safe_radius/2, agent.curr_ss[4] * agent.length / agent0.length- env.safe_radius/2), 
                                  5*env.safe_radius, 5*env.safe_radius, facecolor = 'k', edgecolor = 'k', linestyle = 'dotted', linewidth = 1.5)
            ax.add_patch(rectangle)
            
            circle = Circle((agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length), 
                agent0.guidance_dict['collision_tolerance'], 
                facecolor='none', edgecolor='r', linestyle='dotted', linewidth=1.5)
            
            ax.add_patch(circle)
    
    try:
        for b in env.boundary_list:
            ax.plot([b.start[0]/230, b.end[0]/230],[b.start[1]/230, b.end[1]/230],color='k', linewidth=1.5)        
    except:
        pass

    ylim = ax.get_ylim()
    if ylim[0] - ylim[1] < 20:
        ylim_mean = 0.5*(ylim[1] + ylim[0])
        ylim = [ylim_mean+10, ylim_mean-10]        
        ax.set_ylim(bottom=ylim[0], top=ylim[1])
    
    # ax.set_xlim(left=-10, right=60)
    # ax.set_ylim(bottom=10, top=-10)
    
    # ax.spines['top'].set_visible(False)
    # ax.spines['right'].set_visible(False)
    ax.set_title("Path",fontsize=15)
    ax.set_xlabel("$x/L$",fontsize=15)
    ax.set_ylabel("$y/L$",fontsize=15)
    ax.tick_params(axis='x', labelsize=15)
    ax.tick_params(axis='y', labelsize=15)
    # ax.set_xlim(left=, right=x_max + 10)
    # ax.set_ylim(bottom=y_max + 10, top=y_min - 10)
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    # ax.legend(loc="upper right")
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_path.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_path.svg', format='svg', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_env_path.png', format='png', bbox_inches='tight')
    plt.close()
    
def generate_trajectory_anim(env, case_dir, gif_id):
    
    x_ship = np.array([-0.5, -0.5, 0.25, 0.5, 0.25, -0.5, -0.5, 0.5, 0.25, 0, 0])
    y_ship = 16.1 / 230 * np.array([-1, 1, 1, 0, -1, -1, 0, 0, 1, 1, -1])
    
    fig, ax = plt.subplots()
    ax.invert_yaxis()
    ax.set_aspect('equal')
    
    agent0 = env.agent_list[0]
    
    ship_shape_ax = []
    ship_path_ax = []
    
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0
    
    for agent in env.agent_list:
        
        if agent.design_speed != 0:
            
            x0 = agent.ss[0, 3] * agent.length / agent0.length
            y0 = agent.ss[0, 4] * agent.length / agent0.length
            psi0 = agent.ss[0, 5]
            
            if gif_id is None:
                if x_min > np.min(agent.ss[:, 3] * agent.length / agent0.length): x_min = np.min(agent.ss[:, 3] * agent.length / agent0.length)
                if y_min > np.min(agent.ss[:, 4] * agent.length / agent0.length): y_min = np.min(agent.ss[:, 4] * agent.length / agent0.length)
                if x_max < np.max(agent.ss[:, 3] * agent.length / agent0.length): x_max = np.max(agent.ss[:, 3] * agent.length / agent0.length)
                if y_max < np.max(agent.ss[:, 4] * agent.length / agent0.length): y_max = np.max(agent.ss[:, 4] * agent.length / agent0.length)
            else:
                if gif_id == agent.id:
                    if x_min > np.min(agent.ss[:, 3] * agent.length / agent0.length): x_min = np.min(agent.ss[:, 3] * agent.length / agent0.length)
                    if y_min > np.min(agent.ss[:, 4] * agent.length / agent0.length): y_min = np.min(agent.ss[:, 4] * agent.length / agent0.length)
                    if x_max < np.max(agent.ss[:, 3] * agent.length / agent0.length): x_max = np.max(agent.ss[:, 3] * agent.length / agent0.length)
                    if y_max < np.max(agent.ss[:, 4] * agent.length / agent0.length): y_max = np.max(agent.ss[:, 4] * agent.length / agent0.length)
            
            if agent.id == 0:
                agent_path_ax, = ax.plot(x0, y0, color='blue', label=f'Agent {agent.id}')
            else:
                agent_path_ax, = ax.plot(x0, y0, label=f'Agent {agent.id}')
            
            x_new_ship = x0 + x_ship * np.cos(psi0) - y_ship * np.sin(psi0)
            y_new_ship = y0 + x_ship * np.sin(psi0) + y_ship * np.cos(psi0)
            
            if agent.id == 0:
                agent_shape_ax, = ax.plot(x_new_ship, y_new_ship, color='saddlebrown', label=None)
            else:
                agent_shape_ax, = ax.plot(x_new_ship, y_new_ship, color='green', label=None)
            
            ship_path_ax.append(agent_path_ax)
            ship_shape_ax.append(agent_shape_ax)
            
            if agent.guidance_dict is not None:
                wp = np.array(agent.guidance_dict['waypoints'])
                x_wp = wp[:, 0] * agent.length / agent0.length
                y_wp = wp[:, 1] * agent.length / agent0.length
                
                ax.plot(x_wp, y_wp, '*', markersize=12,label=f'Agent {agent.id} Waypoints')
                for i in range(1, len(x_wp)):
                    # ax.annotate(f'WP {i+1}', xy=(x_wp[i], y_wp[i]), xytext=(x_wp[i] + 4, y_wp[i] + 4))
                    
                    circle = Circle((x_wp[i], y_wp[i]), 
                        agent.guidance_dict['tracking_tolerance'] * agent.length / agent0.length, 
                        facecolor='none', edgecolor='k', linestyle = 'dotted', linewidth = 1.5)
                    
                    ax.add_patch(circle)
            
        else:
            
            # ax.plot(agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length, '*k', 
            #     markersize=12, label=f'Agent {agent.id} - Obstacle')
            
            circle = Circle((agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length), 
                env.safe_radius, 
                facecolor='none', edgecolor='k', linestyle='dotted', linewidth=1.5)
            
            ax.add_patch(circle)
            
            circle = Circle((agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length), 
                agent.length / (2 * agent0.length), 
                facecolor='k', edgecolor='k', linewidth=1.5)
            
            ax.add_patch(circle)
            
            circle = Circle((agent.curr_ss[3] * agent.length / agent0.length, agent.curr_ss[4] * agent.length / agent0.length), 
                agent0.guidance_dict['collision_tolerance'], 
                facecolor='none', edgecolor='r', linestyle='dotted', linewidth=1.5)
            
            ax.add_patch(circle)
    
    agent0 = env.agent_list[0]
    
    x_mean = (x_min + x_max) / 2
    y_mean = (y_min + y_max) / 2
    x_side = x_max - x_min
    y_side = y_max - y_min
    side = max([x_side, y_side])
    x_min = x_mean - side / 2
    x_max = x_mean + side / 2
    y_min = y_mean - side / 2
    y_max = y_mean + side / 2
    
    ax.set_title('Trajectory',fontsize=15)
    ax.set_xlabel("$x/L$",fontsize=15)
    ax.set_ylabel("$y/L$",fontsize=15)
    ax.tick_params(axis='x', labelsize=15)
    ax.tick_params(axis='y', labelsize=15)
    ax.set_xlim(left=x_min - 10, right=x_max + 10)
    ax.set_ylim(bottom=y_max + 10, top=y_min - 10)
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')
    
    def animate(i):
        
        for j, agent in enumerate(env.agent_list):
    
            if agent.design_speed != 0:
                
                x = agent.ss[0:i+1, 3] * agent.length / agent0.length
                y = agent.ss[0:i+1, 4] * agent.length / agent0.length
                psi = agent.ss[0:i+1, 5] 
                
                ship_path_ax[j].set_data(x, y)
                
                x_new_ship = x[i] + x_ship * np.cos(psi[i]) - y_ship * np.sin(psi[i])
                y_new_ship = y[i] + x_ship * np.sin(psi[i]) + y_ship * np.cos(psi[i])
                
                ship_shape_ax[j].set_data(x_new_ship, y_new_ship)
        
        return ship_path_ax + ship_shape_ax
    
    ani = FuncAnimation(fig, animate, interval=35, blit=False, frames=env.n_timesteps)
    ani.save(f'{case_dir}/{case_dir[-7:]}_env_path.gif', writer='pillow')
        

def plot_path(agent, case_dir):
    
    fig, ax = plt.subplots()
    ax.invert_yaxis()
    ax.set_aspect('equal')
    
    x = agent.ss[:, 3]
    y = agent.ss[:, 4]
    
    ax.plot(x, y, label=f'Agent {agent.id} path', linewidth=1.5)
    
    if agent.guidance_dict is not None:
        
        wp = np.array(agent.guidance_dict['waypoints'])
        x_wp = wp[:, 0]
        y_wp = wp[:, 1]
        
        ax.plot(x_wp, y_wp, '*', markersize=12, label='Waypoints', color='red')
        
        for i in range(len(x_wp)):
            circle = Circle((x_wp[i], y_wp[i]), agent.guidance_dict['tracking_tolerance'], facecolor='none', edgecolor='k', linestyle = 'dotted', linewidth = 1.5)
            ax.add_patch(circle)
    
    yl = ax.get_ylim()
    xl = ax.get_xlim()
    
    if np.abs(xl[1] - xl[0]) < 20:
        xl = [xl[0] - 10, xl[1] + 10]
        ax.set_xlim(xl)
    
    if np.abs(yl[1] - yl[0]) < 20:
        yl = [yl[0] + 10, yl[1] - 10]
        ax.set_ylim(yl)
    
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$x/L$',fontsize=15)
    plt.ylabel('$y/L$',fontsize=15)
    plt.title('Waypoint Tracking',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_path.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_path.svg', format='svg', bbox_inches='tight')
    plt.close()

def plot_cross_track_error(agent, case_dir):    
    t = agent.t
    ype = agent.ype
    
    # Ensure both arrays have the same length
    min_length = min(len(t), len(ype))
    t = t[:min_length]
    ype = ype[:min_length]

    fig, ax = plt.subplots()
    ax.plot(t, ype, 'k', linewidth=1.5, label=f'Agent {agent.id}' )
    ax.set_xlim([t[0], t[-1]])
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$t\'$',fontsize=15)
    plt.ylabel('$y_p^{e}$',fontsize=15)
    plt.title('Cross-track error',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(loc='best')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_cross_track_error.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_cross_track_error.svg', format='svg', bbox_inches='tight')
    plt.close()
    
def plot_heading(agent, case_dir):
    
    t = agent.t
    psi = ssa_array(agent.ss[:, 5]) * 180 / np.pi
    psi_des = ssa_array(agent.psi_des) * 180 / np.pi
    
    # Ensure both arrays have the same length
    min_length = min(len(t), len(psi_des))
    t = t[:min_length]
    psi_des = psi_des[:min_length]
    psi = psi[:min_length]

    fig, ax = plt.subplots()
    ax.plot(t, psi_des, '--r', linewidth=1.5, label=f'Agent {agent.id} - Desired' )
    ax.plot(t, psi, 'k', linewidth=1.5, label=f'Agent {agent.id} - Actual' )
    ax.set_xlim([t[0], t[-1]])
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$t\'$',fontsize=15)
    plt.ylabel('$\psi_{des}$ and $\psi$ in deg',fontsize=15)
    plt.title('Heading Angle (Desired vs Actual)',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(loc='best')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_heading.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_heading.svg', format='svg', bbox_inches='tight')
    plt.close()

def plot_delta(agent, case_dir):
    
    t = agent.t
    delta = ssa_array(agent.ss[:, 6]) * 180 / np.pi
    delta_c = ssa_array(agent.delta_c) * 180 / np.pi
    
    # Ensure both arrays have the same length
    min_length = min(len(t), len(delta_c))
    t = t[:min_length]
    delta_c = delta_c[:min_length]
    delta = delta[:min_length]
    
    fig, ax = plt.subplots()
    ax.plot(t, delta_c, '--r', linewidth=1.5, label=f'Agent {agent.id} - Commanded' )
    ax.plot(t, delta, 'k', linewidth=1.5, label=f'Agent {agent.id} - Actual' )
    ax.set_xlim([t[0], t[-1]])
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$t\'$',fontsize=15)
    plt.ylabel('$\delta_{c}$ and $\delta$ in deg',fontsize=15)
    plt.title('Rudder Angle (Commanded vs Actual)',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(loc='best')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_rudder.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_rudder.svg', format='svg', bbox_inches='tight')
    plt.close()
    
    t = agent.t
    n_prop = agent.ss[:, 7] * 60
    n_c = agent.n_c * 60
    
    min_length1 = min(len(t), len(n_c))
    t = t[:min_length1]
    n_c = n_c[:min_length1]
    n_prop = n_prop[:min_length1]
    
    fig, ax = plt.subplots()
    ax.plot(t, n_c, '--r', linewidth=1.5, label=f'Agent {agent.id} - Commanded' )
    ax.plot(t, n_prop, 'k', linewidth=1.5, label=f'Agent {agent.id} - Actual' )
    ax.set_xlim([t[0], t[-1]])
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$t\'$',fontsize=15)
    plt.ylabel('$n_{c}$ and $n$ in RPM',fontsize=15)
    plt.title('Propeller Rate (Commanded vs Actual)',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(loc='best')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_propeller.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_propeller.svg', format='svg', bbox_inches='tight')
    plt.close()

def plot_state_space(agent, case_dir):
    
    t = agent.t
    u = agent.ss[:, 0]
    v = agent.ss[:, 1]
    r = agent.ss[:, 2]
    x = agent.ss[:, 3]
    y = agent.ss[:, 4]
    
    fig, ax = plt.subplots()
    
    min_length1 = min(len(t), len(u))
    t = t[:min_length1]
    u = u[:min_length1]   
    ax.plot(t, u, linewidth=1.5, label=f'Agent {agent.id} - u\'' )
    
    min_length2 = min(len(t), len(v))   
    t = t[:min_length2]
    v = v[:min_length2]    
    ax.plot(t, v, linewidth=1.5, label=f'Agent {agent.id} - v\'' )
    
    min_length3 = min(len(t), len(r))
    t = t[:min_length3]
    r = r[:min_length3]    
    ax.plot(t, r, linewidth=1.5, label=f'Agent {agent.id} - r\'' )
    
    ax.set_xlim([t[0], t[-1]])
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$t\'$',fontsize=15)
    plt.ylabel('$u\'$, $v\'$ and $r\'$',fontsize=15)
    plt.title('Nondimensional Velocities',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(loc='best')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_velocities.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_velocities.svg', format='svg', bbox_inches='tight')
    plt.close()
    
    fig, ax = plt.subplots()
    min_length3 = min(len(t), len(x))
    t = t[:min_length1]
    x = x[:min_length1]  
    ax.plot(t, x, linewidth=1.5, label=f'Agent {agent.id} - x\'' )
    min_length3 = min(len(t), len(y))
    t = t[:min_length2]
    y = y[:min_length2]  
    ax.plot(t, y, linewidth=1.5, label=f'Agent {agent.id} - y\'' )
    ax.set_xlim([t[0], t[-1]])
    ax.grid(linewidth = 0.5, color='gray',linestyle='dotted')        
    plt.xlabel('$t\'$',fontsize=15)
    plt.ylabel('$x/L$ and $y/L$',fontsize=15)
    plt.title('Nondimensional Position',fontsize=15)
    plt.tick_params(axis='x', labelsize=15)
    plt.tick_params(axis='y', labelsize=15)
    plt.legend(loc='best')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_position.pdf', format='pdf', bbox_inches='tight')
    plt.savefig(f'{case_dir}/{case_dir[-8:]}_{agent.id:02d}_position.svg', format='svg', bbox_inches='tight')
    plt.close()