#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 20 16:21:13 2024

@author: mrityunjay
"""

import numpy as np

class Boundary():

    def __init__(self, x0, y0, x1, y1, threshold, strength, id=None):

        self.id = None
        if id is not None:
            self.id = id
        
        self.start = np.array([x0, y0])
        self.end = np.array([x1, y1])

        self.m = (self.end[1] - self.start[1]) / (self.end[0] - self.start[0])
        self.c = self.start[1] - self.m * self.start[0]

        self.psi = np.arctan2(self.end[1] - self.start[1], self.end[0] - self.start[0])

        self.threshold = threshold
        self.strength = strength

        self.grad = np.zeros(2)


    def check_agent_in_range(self, env, agent_id):

        agent_x = env.agent_list[agent_id].curr_ss[3]
        agent_y = env.agent_list[agent_id].curr_ss[4]

        xrel = agent_x - self.start[0] / env.agent_list[agent_id].length
        yrel = agent_y - self.start[1] / env.agent_list[agent_id].length

        xyrel = np.array([xrel, yrel])

        R = np.array([[np.cos(self.psi), np.sin(self.psi)], [-np.sin(self.psi), np.cos(self.psi)]])

        xcyc = R @ xyrel

        xc = xcyc[0]
        yc = xcyc[1]

        grad_c = np.array([0, self.strength/2/np.pi/yc])
        grad = R.T @ grad_c

        # print(f'{agent_x:.2f}, {agent_y:.2f}, {yrel:.2f}, {yc:.2f}, {grad[0]:.2f}, {grad[1]:.2f}')

        if np.abs(yc) > self.threshold:
            return False
        else:
            return True
    
    def get_boundary_grad(self, agent_x, agent_y, agent_length):
        
        xrel = agent_x - self.start[0] / agent_length
        yrel = agent_y - self.start[1] / agent_length

        xyrel = np.array([xrel, yrel])

        R = np.array([[np.cos(self.psi), np.sin(self.psi)], [-np.sin(self.psi), np.cos(self.psi)]])

        xcyc = R @ xyrel

        xc = xcyc[0]
        yc = xcyc[1]

        grad_c = np.array([0, self.strength/2/np.pi/yc])
        grad = R.T @ grad_c       
        
        if np.abs(yc) > self.threshold:
            grad = np.zeros(2)
        
        # print(f'{agent_x:.2f}, {agent_y:.2f}, {yrel:.2f}, {yc:.2f}, {grad[0]:.2f}, {grad[1]:.2f}')

        return grad
        