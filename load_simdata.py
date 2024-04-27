#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 21 15:33:12 2024

@author: mrityunjay
"""

import numpy as np
from plotting import generate_plots
from scipy.io import loadmat, savemat
from env_empty import Environment_empty
import os

def load_simdata_indv(path_str):
    mdict = loadmat(path_str)
    env = Environment_empty(mdict)
    generate_plots(env, gif_id=None, mat_flag=True, plot_flag=True, stat_id=None, mat_save_flag=True)

def load_simdata_stat(path_str, indx):
    indx = indx - 1
    mdict = loadmat(path_str)
    # print(type(mdict['env_data_list'][0, indx][0, 0]), np.shape(mdict['env_data_list'][0, indx][0, 0]))
    # print(mdict['env_data_list'][0, indx][0, 0])
    # print(mdict['env_data_list'][0][indx])
    env = Environment_empty(mdict['env_data_list'][0, indx][0, 0])
    # exit()
    generate_plots(env, gif_id=None, mat_flag=True, plot_flag=True, stat_id=indx, mat_save_flag=True)


if __name__ == "___main__":

    indv_flag = False
    
    if indv_flag:
        case_list = np.arange(29) + 1 
        case_list = case_list.tolist()
        case_list = [29]
        
        for i in case_list:
            path_to_mat = os.path.join(os.getcwd(), f'../plots/case_{i:02d}/simdata.mat')
            load_simdata_indv(path_to_mat)
    
    else:
        # path_to_mat = os.path.join(os.getcwd(), f'../simdata_stat/simdata_stat_05_mvortex.mat')
        path_to_mat = os.path.join(os.getcwd(), f'./simdata_stat_05_mvortex.mat')
        load_simdata_stat(path_to_mat, 4)
    