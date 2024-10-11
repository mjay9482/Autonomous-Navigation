#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May  7 09:54:35 2024

@author: mrityunjay
"""

import numpy as np
from common_params import kcs_params, kcs_guidance, kcs_pid

case_dir = 'case04hrvo'

time_step = 0.25
Tmax = 250

safe_radius = 15

# Agent type (kcs or kinematic) 
# kcs will always be a dynamic agent (moving in a straight line or proactive in collision avoidance)
# kinematic may be dynamic (moving in a straight line only) or static
agent_list = [
    'kcs',
    'kcs',
    'kcs'
    ]

# Agent Size
agent_size = [
    230,
    230,
    230
    ]

# Agent Speed in terms of Froude Number 
# For dynamic vessels make sure propeller RPM is set correctly to achieve this speed
# For kinematic vessels make sure the initial conditions are set correctly to maintain this speed
agent_froude = [
    0.13,
    0.26,
    0.26
    ]

# Initial Conditions (Expressed in non-dimensional format with respect to each agent)
agent_init_state = [
    [1, 0., 0., 0., 0., 0., 0., 58.0/60, 0., 0.],
    [1., 0., 0., -20., 0., 0., 0., 115.5/60, 0., 0.],
    [1., 0., 0., 70., 0., np.pi, 0., 115.5/60, 0., 0.]
    ]

# Agent plant parameters
agent_plant = [
    kcs_params,
    kcs_params,
    kcs_params
    ]

# Control Parameters
agent_controller = [
    kcs_pid,
    kcs_pid,
    kcs_pid
    ]

# Guidance Parameters
kcs1_guidance = kcs_guidance.copy()
kcs1_guidance['waypoints'] = [[0, 0], [60, 0]]
kcs1_guidance['reactive_guidance'] = 'hrvo'

kcs2_guidance = kcs_guidance.copy()
kcs2_guidance['waypoints'] = [[-20, 0], [100, 0]]
kcs2_guidance['reactive_guidance'] = 'hrvo'

kcs3_guidance = kcs_guidance.copy()
kcs3_guidance['waypoints'] = [[70, 0], [10, 0]]
kcs3_guidance['reactive_guidance'] = 'hrvo'

agent_guidance = [
    kcs1_guidance,
    kcs2_guidance,
    kcs3_guidance
    ]