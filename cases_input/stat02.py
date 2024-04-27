#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 25 17:35:28 2024

@author: mrityunjay
"""

import numpy as np
from common_params import kcs_params, kcs_guidance, kcs_pid
from guidance_control import ssa

case_dir = 'stat_02_vo'
np.random.seed()
np.random.seed()

time_step = 0.25
Tmax = 300

safe_radius = 15

nstatic = 2
ndynamic = 5
ntotal = nstatic + ndynamic + 1

# Agent type (kcs or kinematic) 
# kcs will always be a dynamic agent (moving in a straight line or proactive in collision avoidance)
# kinematic may be dynamic (moving in a straight line only) or static

agent_list = []

for i in range(ntotal):
    if i <= ndynamic:
        agent_list.append('kcs')
    else:
        agent_list.append('kinematic')
        
# Agent Size
agent_size = []

for i in range(ntotal):
    if i == 0:
        agent_size.append(230.)
    elif 0 < i <= ndynamic:
        agent_size.append(230.)
        # agent_size.append(230. * (0.5 + np.random.random()/2))
    else:
        agent_size.append(230.)
        # agent_size.append(230. * (0.25 + np.random.random()/2))

# Agent Speed in terms of Froude Number 
# For dynamic vessels make sure propeller RPM is set correctly to achieve this speed
# For kinematic vessels make sure the initial conditions are set correctly to maintain this speed
agent_froude = []

for i in range(ntotal):
    if i == 0:
        agent_froude.append(0.26)
    elif 0 < i <= ndynamic:
        agent_froude.append(0.26 * (0.5 + np.random.random()/2))
    else:
        agent_froude.append(0. * (0.25 + np.random.random()/2) * np.random.randint(2))

# Guidance Parameters
kcs_guidance['reactive_guidance'] = 'vo'

agent_guidance = []

for i in range(ntotal):
    if i <= ndynamic:
        agent_guidance.append(kcs_guidance.copy())
    else:
        agent_guidance.append(None)

waypoints = np.zeros((ntotal, 4))
for i in range(ntotal):
    reset_flag = 1
    while reset_flag == 1:
        temp = np.random.rand(4) * 100 * agent_size[0] / agent_size[i]
        dist = np.sqrt((temp[3] - temp[1]) ** 2 + (temp[2] - temp[0]) **2 )
        while dist < 50:
            temp = np.random.rand(4) * 100 * agent_size[0] / agent_size[i]
            dist = np.sqrt((temp[3] - temp[1]) ** 2 + (temp[2] - temp[0]) **2 )
        reset_flag = 0
        
        if i > ndynamic:
                temp[2:4] = temp[0:2]
        
        for j in range(i):
            start_pts_dist = np.linalg.norm(waypoints[j, 0:2] -  np.array([temp[0], temp[1]]))
            goal_pts_dist = np.linalg.norm(waypoints[j, 2:4] -  np.array([temp[2], temp[3]]))
            if start_pts_dist < 10 or goal_pts_dist < 10:
                reset_flag = 1
                break
        if reset_flag == 1:
            continue
    waypoints[i, :] = np.round(temp)
     
    if i < ndynamic + 1:
        agent_guidance[i]['waypoints'] = [[np.round(temp[0]), np.round(temp[1])], [np.round(temp[2]), np.round(temp[3])]]


# Initial Conditions (Expressed in non-dimensional format with respect to each agent)
agent_init_state = []

for i in range(ntotal):
    if i <= ndynamic:
        wp = agent_guidance[i]['waypoints'][0]
        agent_init_state.append([1, 0., 0., wp[0], wp[1], ssa(2 * np.pi * np.random.random()), 0., 115.5/60, 0., 0.])
    else:
        wp = waypoints[i, :]
        psi = np.arctan2(wp[3] - wp[1], wp[2] - wp[0])
        Fn = agent_froude[i]
        if Fn > 1e-2:
            speed = 1.
        else:
            speed = 0.
        agent_init_state.append([speed, 0., 0., wp[0], wp[1], ssa(psi), 0., 0., 0., 0.])

# Agent plant parameters
agent_plant = []

for i in range(ntotal):
    if i <= ndynamic:
        agent_plant.append(kcs_params)
    else:
        agent_plant.append(None)


# Control Parameters
agent_controller = []

for i in range(ntotal):
    if i <= ndynamic:
        agent_controller.append(kcs_pid)
    else:
        agent_controller.append(None)