#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 28 05:26:39 2023

@author: mrityunjay
"""
# With speed control

import numpy as np
from scipy.integrate import solve_ivp
import agent_ode
import logger as log
import guidance_control
import time

class Agent():
    
    def __init__(self, agent_type='kcs', id=None, length=1, speed=0, Tmax=300, time_step=0.25):
        
        self.id = None
        if id is not None:
            self.id = id
        
        self.state = 0  # State 0 is active and 1 is terminated
        
        self.ss0 = np.zeros(10)
        
        self.curr_ss = self.ss0
        self.timestep_count = 0
        
        self.dt = time_step
        self.Tmax = np.floor(Tmax / self.dt) * self.dt
        self.n_timesteps = int(np.floor(Tmax / self.dt)) + 1
        self.termination_timestep_indx = self.n_timesteps
        self.ss = np.zeros((self.n_timesteps, 10))
        self.ype = np.zeros(self.n_timesteps)
        self.xpe = np.zeros(self.n_timesteps)
        self.psi_des = np.zeros(self.n_timesteps)
        self.v_des = np.zeros((self.n_timesteps,2))
        self.delta_c = np.zeros(self.n_timesteps)
        self.n_c = np.zeros(self.n_timesteps)
        self.t = np.arange(0, self.Tmax + self.dt, self.dt)
        
        self.agent_type = agent_type
        self.length = length
        self.design_speed = speed
        self.Fn = speed / np.sqrt(9.80665 * length)
        self.uerr_int = 0.
        
        if self.agent_type == 'kcs':
            self.ode = agent_ode.kcs_ode
            self.ss0[0] = 1
            self.ss0[7] = 115.5 / 60
        else:
            self.ode = agent_ode.kinematic_ode
        
        self.control_dict = None
        self.guidance_dict = None
        self.waypoints_tracked_indx = 0
        
        self.obstacles = []
        self.boundary_grad = np.zeros(2)
        
        self.wind_flag = 0
        self.wind_speed = 0
        self.wind_dir = 0
        
        self.wave_flag = 0
        self.wave_height = 0
        self.wave_period = 0
        self.wave_dir = 0
        
        self.mcte = 0.
        self.control_effort = 0.
        self.vo_compute_time = 0.
        self.vo_compute_calls = 0
        self.collision_state = 0
        
    def step(self):
        if self.timestep_count + 1 < self.n_timesteps and self.state == 0:
            
            tspan = (self.t[self.timestep_count], self.t[self.timestep_count] + self.dt)
            
            if self.agent_type in ['kcs', 'kinematic']:
            
                if self.agent_type == 'kcs':
                    
                    # GUIDANCE
                    psi_des, v_des, ypd_int, xpe, ype = self.guidance(self.curr_ss, self.design_speed)
                    
                    # CONTROLLER
                    U = np.linalg.norm(self.curr_ss[:2])
                    v_d = np.linalg.norm(v_des)
                    self.uerr_int = self.uerr_int + (v_d-U) * self.dt
                    delta_c, n_c = self.control(psi_des, v_d, self.curr_ss, self.uerr_int)
                    
                    # Store termination time step index
                    if self.state == 1:
                        self.termination_timestep_indx = self.timestep_count + 1
                           
                    # PLANT DYNAMICS             
                    sol = solve_ivp(lambda t, v: self.ode(t, v, delta_c=delta_c, n_c=n_c, Fn=self.Fn, ypd_int=ypd_int, \
                            wind_flag=self.wind_flag, wind_speed=self.wind_speed, wind_dir=self.wind_dir, \
                            wave_flag=self.wave_flag, wave_height=self.wave_height, \
                            wave_period=self.wave_period, wave_dir=self.wave_dir), \
                            tspan, self.curr_ss, t_eval=tspan, dense_output=True)
                
                if self.agent_type == 'kinematic':
                    # PLANT DYNAMICS FOR KINEMATIC AGENT
                    sol = solve_ivp(lambda t, v: self.ode(t, v),
                            tspan, self.curr_ss, t_eval=tspan, dense_output=True)
                    
                    psi_des = sol.y[5, 1]
                    v_des = 0
                    xpe = 0
                    ype = 0
                    delta_c = 0
                    n_c = 0
                
                # UPDATE STATES
                
                self.curr_ss = np.array(sol.y).T[1, :]
                self.timestep_count += 1
                self.ss[self.timestep_count, :] = self.curr_ss
                self.psi_des[self.timestep_count] = psi_des
                self.v_des[self.timestep_count, :] = v_des
                self.xpe[self.timestep_count] = xpe
                self.ype[self.timestep_count] = ype
                self.delta_c[self.timestep_count] = delta_c
                self.n_c[self.timestep_count] = n_c
                self.mcte = np.mean(np.abs(self.ype[0:self.timestep_count]))
                self.control_effort = np.mean(np.abs(self.ss[0:self.timestep_count, 6])) / (35 * np.pi / 180)
            else:
                log.error(f'Agent {self.id} type "{self.agent_type}" is not defined')
                raise Exception(f'Agent {self.id} type "{self.agent_type}" is not defined')
            
        else:
            log.debug(f'Agent {self.id} is in terminated state!')
            
            self.timestep_count += 1
            if self.timestep_count < self.n_timesteps:

                self.curr_ss[0] = 0.0
                self.curr_ss[1] = 0.0
                self.curr_ss[2] = 0.0
                
                self.ss[self.timestep_count, :] = self.curr_ss
                self.psi_des[self.timestep_count] = self.curr_ss[5]
                self.v_des[self.timestep_count, :] = self.curr_ss[:2]
                self.xpe[self.timestep_count] = 0
                self.ype[self.timestep_count] = 0
                self.delta_c[self.timestep_count] = 0
                self.n_c[self.timestep_count] = 0
            
            # exit()
    
    
    def reset(self, ss0=None, control_dict=None, guidance_dict=None, plant_dict=None):
        
        if ss0 is not None:
            self.ss0 = np.array(ss0)
            self.curr_ss = np.array(ss0)
            self.ss[0, :] = np.array(ss0)
            
        self.state = 0
        self.collision_state = 0
        self.timestep_count = 0
        self.mcte = 0.
        self.control_effort = 0.
        self.vo_compute_calls = 0
        self.vo_compute_time = 0.
        self.obstacles = []
        self.boundary_grad = np.zeros(2)
        
        if control_dict is not None:
            self.control_dict = control_dict
        
        if guidance_dict is not None:
            self.guidance_dict = guidance_dict
        
        if plant_dict is not None:
            self.plant_dict = plant_dict
            
            self.wind_flag = self.plant_dict['wind_flag']
            self.wind_speed = self.plant_dict['wind_speed']
            self.wind_dir = self.plant_dict['wind_dir']
            
            self.wave_flag = self.plant_dict['wave_flag']
            self.wave_height = self.plant_dict['wave_height']
            self.wave_period = self.plant_dict['wave_period']
            self.wave_dir = self.plant_dict['wave_dir']
        
    def guidance(self, ss, design_speed):
        
        if self.guidance_dict is not None:
            wp = self.guidance_dict['waypoints']
            
            if self.waypoints_tracked_indx == len(wp) - 1:
                
                # Set the agent to terminated state 
                # when all waypoints are tracked
                
                log.info(f'Terminating Agent {self.id} as all waypoints tracked')
                self.state = 1
                psi_des = ss[5]
                v_des = ss[:2]
                ypd_int = 0
                xpe = 0
                ype = 0
                
            else:
                
                # Track the current goal waypoint
                
                gwp = wp[self.waypoints_tracked_indx + 1]
                iwp = wp[self.waypoints_tracked_indx]
                
                dist_to_gwp = np.sqrt((gwp[0] - ss[3]) ** 2 + (gwp[1] - ss[4]) ** 2)
                
                if dist_to_gwp < self.guidance_dict['tracking_tolerance']:
                    
                    # If goal waypoint has been tracked, shift to tracking the next waypoint
                    
                    log.info(f'Agent {self.id} - Waypoint {self.waypoints_tracked_indx + 1} ({gwp[0]:.2f},{gwp[1]:.2f}) tracked!')
                    
                    self.waypoints_tracked_indx += 1
                    
                    if self.waypoints_tracked_indx == len(wp) - 1:
                        
                        self.state = 1
                        
                    else:
                        
                        iwp = gwp
                        gwp = wp[self.waypoints_tracked_indx + 1]
                
                dLA = self.guidance_dict['dLA']
                ki = self.guidance_dict['ki']
                kappa = self.guidance_dict['kappa']
                
                # ILOS guidance
                
                psi_des, v_des, ypd_int, xpe, ype = guidance_control.ilos_guidance(ss, iwp=iwp, gwp=gwp, dLA=dLA, ki=ki, kappa=kappa)
                
                # Reactive guidance when obstacles are present inside safe radius
                # Only changes psi_des. ypd_int, xpe and ype are not changed by this call
                
                if self.obstacles or np.linalg.norm(self.boundary_grad) > 1e-3:

                    vo_start = time.time()
                    psi_des, v_des, collision_flag = guidance_control.reactive_guidance(gwp, design_speed, self.obstacles, self.guidance_dict, boundary_grad=self.boundary_grad)
                    vo_end = time.time()
                    
                    self.vo_compute_calls += 1
                    self.vo_compute_time = (self.vo_compute_time * (self.vo_compute_calls - 1) + (vo_end - vo_start)) / self.vo_compute_calls
                
                    if collision_flag == 1:
                        self.state = 1
                        self.collision_state = 1
                    
        else:            
            # Keep tracking the current heading
            
            log.debug(f'Guidance type not set for Agent {self.id}. Guidance will follow current heading!')
            psi_des = ss[5]
            v_des = ss[:2]
            ypd_int = 0
            xpe = 0
            ype = 0
        
        return psi_des, v_des, ypd_int, xpe, ype
    
    
    def control(self, psid, vd, ss, uerr_int):
        
        if self.control_dict is not None:
        
            delta_c, n_c = guidance_control.control(psid, vd, ss, uerr_int, self.obstacles, control_dict = self.control_dict)
                
        else:
            
            log.debug(f'Controller type not set for Agent {self.id}. Control input will be set to zero!')
            delta_c = 0
            n_c =115.5 / 60  # commanded propeller speed (1:75.5 scaled KCS to achieve 1.42 m/s in steady state)
    
        return delta_c, n_c
        

# Without speed control

# import numpy as np
# from scipy.integrate import solve_ivp
# import agent_ode
# import logger as log
# import guidance_control
# import time

# class Agent():
    
#     def __init__(self, agent_type='kcs', id=None, length=1, speed=0, Tmax=300, time_step=0.25):
        
#         self.id = None
#         if id is not None:
#             self.id = id
        
#         self.state = 0  # State 0 is active and 1 is terminated
        
#         self.ss0 = np.zeros(10)
        
#         self.curr_ss = self.ss0
#         self.timestep_count = 0
        
#         self.dt = time_step
#         self.Tmax = np.floor(Tmax / self.dt) * self.dt
#         self.n_timesteps = int(np.floor(Tmax / self.dt)) + 1
#         self.termination_timestep_indx = self.n_timesteps
#         self.ss = np.zeros((self.n_timesteps, 10))
#         self.ype = np.zeros(self.n_timesteps)
#         self.xpe = np.zeros(self.n_timesteps)
#         self.psi_des = np.zeros(self.n_timesteps)
#         self.delta_c = np.zeros(self.n_timesteps)
#         self.n_c = np.zeros(self.n_timesteps)
#         self.t = np.arange(0, self.Tmax + self.dt, self.dt)
        
#         self.agent_type = agent_type
#         self.length = length
#         self.design_speed = speed
#         self.Fn = speed / np.sqrt(9.80665 * length)
        
#         if self.agent_type == 'kcs':
#             self.ode = agent_ode.kcs_ode
#             self.ss0[0] = 1
#             self.ss0[7] = 115.5 / 60
#         else:
#             self.ode = agent_ode.kinematic_ode
        
#         self.control_dict = None
#         self.guidance_dict = None
#         self.waypoints_tracked_indx = 0
        
#         self.obstacles = []
#         self.boundary_grad = np.zeros(2)
        
#         self.wind_flag = 0
#         self.wind_speed = 0
#         self.wind_dir = 0
        
#         self.wave_flag = 0
#         self.wave_height = 0
#         self.wave_period = 0
#         self.wave_dir = 0
        
#         self.mcte = 0.
#         self.control_effort = 0.
#         self.vo_compute_time = 0.
#         self.vo_compute_calls = 0
#         self.collision_state = 0
        
#     def step(self):
#         if self.timestep_count + 1 < self.n_timesteps and self.state == 0:
            
#             tspan = (self.t[self.timestep_count], self.t[self.timestep_count] + self.dt)
            
#             if self.agent_type in ['kcs', 'kinematic']:
            
#                 if self.agent_type == 'kcs':
                    
#                     # GUIDANCE
#                     psi_des, ypd_int, xpe, ype = self.guidance(self.curr_ss)
                    
#                     # CONTROLLER
#                     delta_c, n_c = self.control(psi_des, self.curr_ss)
                    
#                     # Store termination time step index
#                     if self.state == 1:
#                         self.termination_timestep_indx = self.timestep_count + 1
                    
#                     # PLANT DYNAMICS             
#                     sol = solve_ivp(lambda t, v: self.ode(t, v, delta_c=delta_c, n_c=n_c, Fn=self.Fn, ypd_int=ypd_int, \
#                             wind_flag=self.wind_flag, wind_speed=self.wind_speed, wind_dir=self.wind_dir, \
#                             wave_flag=self.wave_flag, wave_height=self.wave_height, \
#                             wave_period=self.wave_period, wave_dir=self.wave_dir), \
#                             tspan, self.curr_ss, t_eval=tspan, dense_output=True)
                
#                 if self.agent_type == 'kinematic':
#                     # PLANT DYNAMICS FOR KINEMATIC AGENT
#                     sol = solve_ivp(lambda t, v: self.ode(t, v),
#                             tspan, self.curr_ss, t_eval=tspan, dense_output=True)
                    
#                     psi_des = sol.y[5, 1]
#                     xpe = 0
#                     ype = 0
#                     delta_c = 0
#                     n_c = 0
                
#                 # UPDATE STATES
                
#                 self.curr_ss = np.array(sol.y).T[1, :]
#                 self.timestep_count += 1
#                 self.ss[self.timestep_count, :] = self.curr_ss
#                 self.psi_des[self.timestep_count] = psi_des
#                 self.xpe[self.timestep_count] = xpe
#                 self.ype[self.timestep_count] = ype
#                 self.delta_c[self.timestep_count] = delta_c
#                 self.n_c[self.timestep_count] = n_c
#                 self.mcte = np.mean(np.abs(self.ype[0:self.timestep_count]))
#                 self.control_effort = np.mean(np.abs(self.ss[0:self.timestep_count, 6])) / (35 * np.pi / 180)
#             else:
#                 log.error(f'Agent {self.id} type "{self.agent_type}" is not defined')
#                 raise Exception(f'Agent {self.id} type "{self.agent_type}" is not defined')
            
#         else:
#             log.debug(f'Agent {self.id} is in terminated state!')
            
#             self.timestep_count += 1
#             if self.timestep_count < self.n_timesteps:

#                 self.curr_ss[0] = 0.0
#                 self.curr_ss[1] = 0.0
#                 self.curr_ss[2] = 0.0
                
#                 self.ss[self.timestep_count, :] = self.curr_ss
#                 self.psi_des[self.timestep_count] = self.curr_ss[5]
#                 self.xpe[self.timestep_count] = 0
#                 self.ype[self.timestep_count] = 0
#                 self.delta_c[self.timestep_count] = 0
#                 self.n_c[self.timestep_count] = 0
            
#             # exit()
    
    
#     def reset(self, ss0=None, control_dict=None, guidance_dict=None, plant_dict=None):
        
#         if ss0 is not None:
#             self.ss0 = np.array(ss0)
#             self.curr_ss = np.array(ss0)
#             self.ss[0, :] = np.array(ss0)
            
#         self.state = 0
#         self.collision_state = 0
#         self.timestep_count = 0
#         self.mcte = 0.
#         self.control_effort = 0.
#         self.vo_compute_calls = 0
#         self.vo_compute_time = 0.
#         self.obstacles = []
#         self.boundary_grad = np.zeros(2)
        
#         if control_dict is not None:
#             self.control_dict = control_dict
        
#         if guidance_dict is not None:
#             self.guidance_dict = guidance_dict
        
#         if plant_dict is not None:
#             self.plant_dict = plant_dict
            
#             self.wind_flag = self.plant_dict['wind_flag']
#             self.wind_speed = self.plant_dict['wind_speed']
#             self.wind_dir = self.plant_dict['wind_dir']
            
#             self.wave_flag = self.plant_dict['wave_flag']
#             self.wave_height = self.plant_dict['wave_height']
#             self.wave_period = self.plant_dict['wave_period']
#             self.wave_dir = self.plant_dict['wave_dir']
    
    
#     def guidance(self, ss):
        
#         if self.guidance_dict is not None:
#             wp = self.guidance_dict['waypoints']
            
#             if self.waypoints_tracked_indx == len(wp) - 1:
                
#                 # Set the agent to terminated state 
#                 # when all waypoints are tracked
                
#                 log.info(f'Terminating Agent {self.id} as all waypoints tracked')
#                 self.state = 1
#                 psi_des = ss[5]
#                 ypd_int = 0
#                 xpe = 0
#                 ype = 0
                
#             else:
                
#                 # Track the current goal waypoint
                
#                 gwp = wp[self.waypoints_tracked_indx + 1]
#                 iwp = wp[self.waypoints_tracked_indx]
                
#                 dist_to_gwp = np.sqrt((gwp[0] - ss[3]) ** 2 + (gwp[1] - ss[4]) ** 2)
                
#                 if dist_to_gwp < self.guidance_dict['tracking_tolerance']:
                    
#                     # If goal waypoint has been tracked, shift to tracking the next waypoint
                    
#                     log.info(f'Agent {self.id} - Waypoint {self.waypoints_tracked_indx + 1} ({gwp[0]:.2f},{gwp[1]:.2f}) tracked!')
                    
#                     self.waypoints_tracked_indx += 1
                    
#                     if self.waypoints_tracked_indx == len(wp) - 1:
                        
#                         self.state = 1
                        
#                     else:
                        
#                         iwp = gwp
#                         gwp = wp[self.waypoints_tracked_indx + 1]
                
#                 dLA = self.guidance_dict['dLA']
#                 ki = self.guidance_dict['ki']
#                 kappa = self.guidance_dict['kappa']
                
#                 # ILOS guidance
                
#                 psi_des, ypd_int, xpe, ype = guidance_control.ilos_guidance(ss, iwp=iwp, gwp=gwp, dLA=dLA, ki=ki, kappa=kappa)
                
#                 # Reactive guidance when obstacles are present inside safe radius
#                 # Only changes psi_des. ypd_int, xpe and ype are not changed by this call
                
#                 if self.obstacles or np.linalg.norm(self.boundary_grad) > 1e-3:
                    
#                     vo_start = time.time()
#                     psi_des, collision_flag = guidance_control.reactive_guidance(gwp, self.obstacles, self.guidance_dict, boundary_grad=self.boundary_grad)
#                     vo_end = time.time()
                    
#                     self.vo_compute_calls += 1
#                     self.vo_compute_time = (self.vo_compute_time * (self.vo_compute_calls - 1) + (vo_end - vo_start)) / self.vo_compute_calls
                
#                     if collision_flag == 1:
#                         self.state = 1
#                         self.collision_state = 1
                    
#         else:
            
#             # Keep tracking the current heading
            
#             log.debug(f'Guidance type not set for Agent {self.id}. Guidance will follow current heading!')
#             psi_des = ss[5]
#             ypd_int = 0
#             xpe = 0
#             ype = 0
        
#         return psi_des, ypd_int, xpe, ype
    
    
#     def control(self, psid, ss):
        
#         if self.control_dict is not None:
        
#             delta_c, n_c = guidance_control.control(psid, ss, control_dict=self.control_dict)
#         else:
            
#             log.debug(f'Controller type not set for Agent {self.id}. Control input will be set to zero!')
#             delta_c = 0
#             n_c =115.5 / 60  # commanded propeller speed (1:75.5 scaled KCS to achieve 1.42 m/s in steady state)
    
#         return delta_c, n_c
        