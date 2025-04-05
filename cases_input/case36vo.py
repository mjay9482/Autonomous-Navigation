import numpy as np
from common_params import kcs_params, kcs_guidance, kcs_pid

case_dir = 'case36vo'

time_step = 0.25
Tmax = 150

safe_radius = 15

# Agent type (kcs or kinematic) 
# kcs will always be a dynamic agent (moving in a straight line or proactive in collision avoidance)
# kinematic may be dynamic (moving in a straight line only) or static
agent_list = [
    'kcs',
    'kinematic'
    ]

# Agent Size
agent_size = [
    230,
    230
    ]

# Agent Speed in terms of Froude Number 
# For dynamic vessels make sure propeller RPM is set correctly to achieve this speed
# For kinematic vessels make sure the initial conditions are set correctly to maintain this speed
agent_froude = [
    0.26,
    0.
    ]

# Initial Conditions (Expressed in non-dimensional format with respect to each agent)
agent_init_state = [
    [1, 0., 0., 0., 0., 0., 0., 115.5/60, 0., 0.],
    [0., 0., 0., 50., 0., 0., 0., 0., 0., 0.]
    ]

# Agent plant parameters
agent_plant = [
    kcs_params,
    None
    ]

# Control Parameters
agent_controller = [
    kcs_pid,
    None
    ]

# Guidance Parameters
kcs1_guidance = kcs_guidance.copy()
kcs1_guidance['waypoints'] = [[0, 0], [100, 0]]
kcs1_guidance['reactive_guidance'] = 'vo'


agent_guidance = [
    kcs1_guidance,
    None
    ]
