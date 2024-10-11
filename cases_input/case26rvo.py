#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 28 16:19:19 2024

@author: mrityunjay
"""

import numpy as np
from common_params import kcs_params, kcs_guidance, kcs_pid

case_dir = 'case26rvo'
np.random.seed()
np.random.seed()

time_step = 0.25
Tmax = 120

safe_radius = 15

# Agent type (kcs or kinematic) 
# kcs will always be a dynamic agent (moving in a straight line or proactive in collision avoidance)
# kinematic may be dynamic (moving in a straight line only) or static
agent_list = [
    'kcs',
    'kcs'
    ]

# Agent Size
agent_size = [
    230,
    230]
# Agent Speed in terms of Froude Number 
# For dynamic vessels make sure propeller RPM is set correctly to achieve this speed
# For kinematic vessels make sure the initial conditions are set correctly to maintain this speed
agent_froude = [
    0.26,
    0.26
    ]

# Initial Conditions (Expressed in non-dimensional format with respect to each agent)
agent_init_state = [
    [1, 0., 0., 20., 50., -np.pi/2, 0., 115.5/60, 0., 0.],
    [1, 0., 0., 20., -50., np.pi/2, 0., 115.5/60, 0., 0.]
    ]
    

# Agent plant parameters
agent_plant = [
    kcs_params,
    kcs_params
    ]

# Control Parameters
agent_controller = [
    kcs_pid,
    kcs_pid
    ]

# Guidance Parameters
kcs1_guidance = kcs_guidance.copy()
kcs1_guidance['waypoints'] = [[20, 50], [20, -50]]
kcs1_guidance['reactive_guidance'] = 'rvo'

kcs2_guidance = kcs_guidance.copy()
kcs2_guidance['waypoints'] = [[20, -50], [20, 50]]
kcs2_guidance['reactive_guidance'] = 'rvo'

agent_guidance = [
    kcs1_guidance,
    kcs2_guidance
    ]
