#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 23 14:57:42 2024

@author: mrityunjay
"""

import run
import numpy as np
from sim_params import sim_params


params = sim_params(seed=0, case_dir=f'case01rvo', method='rvo', environment=1)
params.initialize()

case_list = np.arange(25) + 1 
case_list = case_list.tolist()

# case_list = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
case_list = [0]
for i in case_list:
    exec(f'import cases_input.case{i:02d}rvo as params')
    _ = run.execute(params, gif_id=None, plot_flag=True, mat_flag=True, mat_save_flag=True, plot_speed_control=True)