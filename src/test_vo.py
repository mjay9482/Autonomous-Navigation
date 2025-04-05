#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 15:30:32 2024

@author: mrityunjay
"""

import run
import numpy as np
from sim_params import sim_params


params = sim_params(seed=0, case_dir=f'stat01', method='vo', environment=1)
params.initialize()

case_list = np.arange(25) + 1 
case_list = case_list.tolist()

# case_list = [10, 11, 12, 13, 36, 37, 38, 39]
# case_list = [13, 36]
# case_list = [40, 41, 42, 43, 44, 45, 46, 47]
# case_list = [9, 10, 11, 12, 13, 26, 30, 31, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47]
# case_list = [40, 41, 42, 43, 44, 45, 46, 47]

# case_list = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24]
case_list =[0]
## static
# case_list = [27, 36]
## static with head on 
# case_list = [31,35]
## head on
# case_list = [0,32]
## crossing stbd
# case_list = [2,26]
## overtaking 
for i in case_list:
    exec(f'import cases_input.case{i:02d}vo as params')
    _ = run.execute(params, gif_id=None, plot_flag=True, mat_flag=True, mat_save_flag=True, plot_speed_control=False)