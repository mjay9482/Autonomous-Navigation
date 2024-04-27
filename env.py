#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 19 18:07:18 2024

@author: mrityunjay
"""
     
import numpy as np
from agent import Agent
from obstacle import Obstacle
from boundary import Boundary
import logger as log

class Environment():
    
    def __init__(self, params):
        
        self.params = params
        self.timestep_count = 0
        self.dt = params.time_step
        self.Tmax = np.floor(params.Tmax / self.dt) * self.dt
        self.n_timesteps = int(np.floor(params.Tmax / self.dt)) + 1
        self.t = np.arange(0, self.Tmax + self.dt, self.dt)
        
        self.agent_list = []
        self.boundary_list = []
        
        for i, agent_type in enumerate(params.agent_list):
            length = params.agent_size[i]
            speed = params.agent_froude[i] * np.sqrt(9.80665 * length)
            
            if i == 0 or speed == 0.0:
                time_factor = 1
            else:
                length0 = params.agent_size[0]
                speed0 = params.agent_froude[0] * np.sqrt(9.80665 * length0)
                time_factor = (length0 / speed0) * (speed / length)
            
            indv_agent = Agent(agent_type, id=i, length=length, speed=speed, 
                Tmax=params.Tmax * time_factor, time_step=params.time_step * time_factor)
            
            self.agent_list.append(indv_agent)
           
        self.inter_agent_dist = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_heading = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_vector_dir = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_rel_vel_rad = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_rel_vel_tan = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_tcpa = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_dcpa = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.inter_agent_theta = np.zeros((len(self.agent_list), len(self.agent_list), self.n_timesteps))
        self.safe_radius = params.safe_radius
        
        try:
            boundaries = params.boundary_list            
            for i, b in enumerate(boundaries):
                bound = Boundary(b[0], b[1], b[2], b[3], b[4], b[5], i)
                self.boundary_list.append(bound)

        except:
            pass
        
    def step(self):
        
        if self.timestep_count + 1 < self.n_timesteps:
    
            for i, agent in enumerate(self.agent_list):
                agent.step()
                agent.obstacles = []
            
            self.timestep_count += 1
            self.evaluate_obstacles()
        
        else:
            log.info(f'Timestep -> {self.timestep_count}, Max Timesteps -> {self.n_timesteps}')
            log.warn('Exceeded total time steps specified in params.py. No agent steps forward!')
    
    def reset(self):
        for i in range(len(self.agent_list)):
            self.agent_list[i].reset(ss0=self.params.agent_init_state[i], 
                control_dict=self.params.agent_controller[i], 
                guidance_dict=self.params.agent_guidance[i],
                plant_dict=self.params.agent_plant[i])
        
        self.timestep_count = 0
        self.evaluate_obstacles()
    
    def evaluate_obstacles(self):
        
        k = self.timestep_count
        
        for i in range(len(self.agent_list)):
            for j in range(i):
                
                if i != j:
                    
                    agent1 = self.agent_list[i]
                    agent2 = self.agent_list[j]
                    
                    obs_12 = Obstacle(self, agent_id=i, obs_id=j)
                    obs_21 = Obstacle(self, agent_id=j, obs_id=i)
                    
                    if obs_12.dist < self.safe_radius:
                        agent1.obstacles.append(obs_12)
                        
                    if obs_21.dist < self.safe_radius:
                        agent2.obstacles.append(obs_21)
                        
                    self.inter_agent_dist[i, j, k] = obs_12.dist
                    self.inter_agent_dist[j, i, k] = obs_21.dist
                    self.inter_agent_heading[i, j, k] = obs_12.rel_heading
                    self.inter_agent_heading[j, i, k] = obs_21.rel_heading
                    self.inter_agent_vector_dir[i, j, k] = obs_12.gamma
                    self.inter_agent_vector_dir[j, i, k] = obs_21.gamma
                    self.inter_agent_rel_vel_rad[i, j, k] = obs_12.rel_vel_rad
                    self.inter_agent_rel_vel_rad[j, i, k] = obs_21.rel_vel_rad
                    self.inter_agent_rel_vel_tan[i, j, k] = obs_12.rel_vel_tan
                    self.inter_agent_rel_vel_tan[j, i, k] = obs_21.rel_vel_tan
                    self.inter_agent_tcpa[i, j, k] = obs_12.tcpa
                    self.inter_agent_tcpa[j, i, k] = obs_21.tcpa
                    self.inter_agent_dcpa[i, j, k] = obs_12.dcpa
                    self.inter_agent_dcpa[j, i, k] = obs_21.dcpa
                    self.inter_agent_theta[i, j, k] = obs_12.theta
                    self.inter_agent_theta[j, i, k] = obs_21.theta
            
            if self.boundary_list:

                grad = np.zeros(2)

                for j in range(len(self.boundary_list)):

                    if self.boundary_list[j].check_agent_in_range(self, i):
                        print('Boundary effect felt')
                        
                        b = self.boundary_list[j]
                        
                        agent_x = self.agent_list[i].curr_ss[3]
                        agent_y = self.agent_list[i].curr_ss[4]
                        agent_length = self.agent_list[i].length

                        grad_temp = b.get_boundary_grad(agent_x, agent_y, agent_length)
                        
                        grad += grad_temp
                        
                self.agent_list[i].boundary_grad = grad