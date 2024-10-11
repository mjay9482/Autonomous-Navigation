#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 15:08:05 2024

@author: mrityunjay
"""

kcs_params = {
    'wind_flag': 0,
    'wind_speed' : 0,
    'wind_dir' : 0,
    'wave_flag' : 0,
    'wave_height' : 0,
    'wave_period' : 0,
    'wave_dir' : 0
}

kcs_pid = {
    'control_type':'PD', 
    'Kp': 3.5, 
    'Kd': 4.0
}

kcs_guidance = {
    'waypoints': None, 
    'dLA' : 4,
    'ki' : 0.025,
    'kappa' : 0.5,
    'tracking_tolerance' : 3,
    'collision_tolerance' : 2,
    'reactive_guidance' : 'vo',
    'katt' : 50,
    'krep' : 200000,
    'vortex_strength' : 10,
    'source_strength' : 100,
    'sink_strength' : -100,
    'doublet_strength' : 100
}