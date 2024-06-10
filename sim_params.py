#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 15:07:12 2024

@author: mrityunjay
"""

import numpy as np
from common_params import kcs_params, kcs_guidance, kcs_pid
from guidance_control import ssa

class sim_params():
    
    def __init__(self,
        case_dir='stat_01', 
        time_step = 0.25,
        Tmax = 300,
        nstatic = 0, 
        ndynamic = 0,
        seed = 0,
        method = 'rvo',
        environment = None,
        safe_radius = 15
        ):
        
        self.case_dir = case_dir
        self.seed = seed
        self.time_step = time_step
        self.Tmax = Tmax
        
        self.nstatic = nstatic
        self.ndynamic = ndynamic
        
        
        self.environment = environment
        if self.environment is not None:
            if self.environment == 1:
                self.nstatic = 1
                self.ndynamic = 2
            elif self.environment == 2:
                self.nstatic = 2
                self.ndynamic = 3
            elif self.environment == 3:
                self.nstatic = 2
                self.ndynamic = 5
            elif self.environment == 4:
                self.nstatic = 3
                self.ndynamic = 7
            elif self.environment == 5:
                self.nstatic = 4
                self.ndynamic = 9
            else:
                pass
        
        self.safe_radius = safe_radius
        self.method = method
        
        self.agent_list = []
        self.agent_size = []
        self.agent_froude = []
        
        self.agent_plant = []
        self.agent_guidance = []
        self.agent_controller = []
        self.agent_init_state = []
        
        self.waypoints = None
    
    def reload(self, seed):
        self.seed = seed
        
        self.agent_list = []
        self.agent_size = []
        self.agent_froude = []
        
        self.agent_plant = []
        self.agent_guidance = []
        self.agent_controller = []
        self.agent_init_state = []
        
        # self.initialize()
        return self
    
    def initialize(self):
        
        np.random.seed(self.seed)
        
        self.ntotal = self.nstatic + self.ndynamic + 1

        for i in range(self.ntotal):
            if i <= self.ndynamic:
                self.agent_list.append('kcs')
                
            else:
                self.agent_list.append('kinematic')
    
        for i in range(self.ntotal):
            if i == 0:
                self.agent_size.append(230.)
            elif 0 < i <= self.ndynamic:
                self.agent_size.append(230.)
                # self.agent_size.append(230. * (0.5 + np.random.random()/2))
            else:
                self.agent_size.append(230.)
                # self.agent_size.append(230. * (0.25 + np.random.random()/2))

        for i in range(self.ntotal):
            if i == 0:
                self.agent_froude.append(0.26)
            elif 0 < i <= self.ndynamic:
                self.agent_froude.append(0.26 * (0.5 + np.random.random()/2))
            else:
                self.agent_froude.append(0. * (0.25 + np.random.random()/2) * np.random.randint(2))
        
        # Guidance Parameters
        kcs_guidance['reactive_guidance'] = self.method
        
        for i in range(self.ntotal):
            if i <= self.ndynamic:
                self.agent_guidance.append(kcs_guidance.copy())
            else:
                self.agent_guidance.append(None)
        
        self.waypoints = np.zeros((self.ntotal, 4))
        for i in range(self.ntotal):
            reset_flag = 1
            while reset_flag == 1:
                temp = np.random.rand(4) * 100 * self.agent_size[0] / self.agent_size[i]
                dist = np.sqrt((temp[3] - temp[1]) ** 2 + (temp[2] - temp[0]) **2 )
                while dist < 50:
                    temp = np.random.rand(4) * 100 * self.agent_size[0] / self.agent_size[i]
                    dist = np.sqrt((temp[3] - temp[1]) ** 2 + (temp[2] - temp[0]) **2 )
                reset_flag = 0
                
                if i > self.ndynamic:
                        temp[2:4] = temp[0:2]
                
                for j in range(i):
                    start_pts_dist = np.linalg.norm(self.waypoints[j, 0:2] -  np.array([temp[0], temp[1]]))
                    goal_pts_dist = np.linalg.norm(self.waypoints[j, 2:4] -  np.array([temp[2], temp[3]]))
                    if start_pts_dist < 10 or goal_pts_dist < 10:
                        reset_flag = 1
                        break
                if reset_flag == 1:
                    continue
            self.waypoints[i, :] = np.round(temp)
             
            if i < self.ndynamic + 1:
                self.agent_guidance[i]['waypoints'] = [[np.round(temp[0]), np.round(temp[1])], [np.round(temp[2]), np.round(temp[3])]]

        for i in range(self.ntotal):
            if i <= self.ndynamic:
                wp = self.agent_guidance[i]['waypoints'][0]
                self.agent_init_state.append([1, 0., 0., wp[0], wp[1], ssa(2 * np.pi * np.random.random()), 0., 115.5/60, 0., 0.])
                
            else:
                wp = self.waypoints[i, :]
                psi = np.arctan2(wp[3] - wp[1], wp[2] - wp[0])
                Fn = self.agent_froude[i]
                if Fn > 1e-2:
                    speed = 1.
                else:
                    speed = 0.
                self.agent_init_state.append([speed, 0., 0., wp[0], wp[1], ssa(psi), 0., 0., 0., 0.])
        
        # Agent plant parameters
        for i in range(self.ntotal):
            if i <= self.ndynamic:
                self.agent_plant.append(kcs_params)
            else:
                self.agent_plant.append(None)
        

        # Control Parameters
        for i in range(self.ntotal):
            if i <= self.ndynamic:
                self.agent_controller.append(kcs_pid)
            else:
                self.agent_controller.append(None)
        
        