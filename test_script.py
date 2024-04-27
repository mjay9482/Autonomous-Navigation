#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 15:09:54 2024

@author: mrityunjay
"""

import numpy as np
from sim_params import sim_params
import run

def print_wp(params):
    print(len(params.agent_guidance))

params = sim_params(seed=0, case_dir=f'stat_01', method='vo', environment=1)
params.initialize()

for i in range(1,0,-1):
    print_wp(params.reload(i+1))
    _ = run.execute(params.reload(i+1), gif_id=None, plot_flag=True, mat_flag=True, mat_save_flag=True, stat_id=i)