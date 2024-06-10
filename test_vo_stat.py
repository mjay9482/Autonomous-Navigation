#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 25 17:34:46 2024

@author: mrityunjay
"""

import run
import numpy as np
from sim_params import sim_params

params = sim_params(seed=0, case_dir=f'stat02', method='vo', environment=1)
params.initialize()

case_list = np.arange(1) + 1 
case_list = case_list.tolist()

# case_list = [10, 11, 12, 13, 36, 37, 38, 39]
# case_list = [13, 36]
# case_list = [40, 41, 42, 43, 44, 45, 46, 47]
# case_list = [9, 10, 11, 12, 13, 26, 30, 31, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47]
# case_list = [40, 41, 42, 43, 44, 45, 46, 47]

case_list = [4]

for i in case_list:
    #exec(f'import cases_input.stat{i:02d} as params')
    exec(f'import cases_input.stat{i:02d} as params')
    _ = run.execute(params, gif_id=None, plot_flag=True, mat_flag=True, mat_save_flag=True)
    
