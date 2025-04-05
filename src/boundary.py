# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Created on Sat Apr 20 16:21:13 2024

# @author: mrityunjay
# """

import numpy as np
from matplotlib.path import Path
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import LinearRing

class Boundary():

    def __init__(self, x0, y0, x1, y1, threshold, strength, boundary_list, id=None):

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
        self.boundary_list = boundary_list
        # self.goal = goal
    
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
    
    def get_boundary_grad(self, agent_x, agent_y, agent_length, agent_x_vel, agent_y_vel, goal):
        
        xrel = agent_x - self.start[0] / agent_length
        yrel = agent_y - self.start[1] / agent_length

        xyrel = np.array([xrel, yrel])

        R = np.array([[np.cos(self.psi), np.sin(self.psi)], [-np.sin(self.psi), np.cos(self.psi)]])

        xcyc = R @ xyrel

        xc = xcyc[0]
        yc = xcyc[1]
               
        vel = np.array([agent_x_vel, agent_y_vel])
        v_norm = np.linalg.norm(vel)
        v_norm = 1.
    
        curr_pos = np.array([agent_x, agent_y])
        
        if np.array_equal(self.start, goal):  
            vel_d = np.array([0, 0])
        else: 
            vel_d = ((goal - curr_pos) / np.linalg.norm(goal - curr_pos)) * v_norm 
        dt = 1
 
        num_points1 = 150
        num_points2 = 150
        thickness = -self.threshold/2
        
        v_new = []      
        boundary_points = []
        
        l = agent_length
        scaled_boundary = [[x0/l,y0/l,x1/l,y1/l,threshold,strength]
                     for x0,y0,x1,y1,threshold,strength in self.boundary_list]
        
        for i in range(len(self.boundary_list)):
            boundary_points.append([scaled_boundary[i][0],scaled_boundary[i][1]])
        
        outer_boundary_points, inner_boundary_points = self.pad_boundaries(boundary_points, thickness)
        codes_outer = [Path.MOVETO] + [Path.LINETO] * (len(outer_boundary_points) - 2) + [Path.CLOSEPOLY]
        codes_inner = [Path.MOVETO] + [Path.LINETO] * (len(inner_boundary_points) - 2) + [Path.CLOSEPOLY]

        outer_path = Path(outer_boundary_points, codes_outer)
        inner_path = Path(inner_boundary_points, codes_inner)
        
        mag1 = np.linspace(1., v_norm + 1., num_points2)
        mag2 = np.linspace(1., v_norm + 1., num_points2)
        theta1 = np.linspace(self.psi + np.pi, self.psi - np.pi, num_points1)
        theta2 = np.linspace(self.psi + np.pi, self.psi - np.pi, num_points1)

        for (m1, m2), (t1, t2) in zip(zip(mag1, mag2), zip(theta1, theta2)):
            new_vel = np.array([m1 * np.cos(t1) , m2 * np.sin(t2)])
        # for theta in np.linspace(self.psi + np.pi, self.psi - np.pi, num_points1):
        #     for mag in np.linspace(0.5, v_norm + 0.5, num_points2):
                # x_component = mag * np.cos(theta)
                # y_component = mag * np.sin(theta)
                # new_vel = np.array([x_component, y_component])  
            curr_pos = np.array([agent_x, agent_y])
            final_pos = curr_pos + new_vel * dt
            
            if inner_path.contains_point([final_pos[0], curr_pos[1]]) and outer_path.contains_point([final_pos[0], curr_pos[1]]):
                valid_x = True
            else:
                valid_x = False

            if inner_path.contains_point([curr_pos[0], final_pos[1]]) and outer_path.contains_point([curr_pos[0], final_pos[1]]):
                valid_y = True
            else:
                valid_y = False

            if valid_x and valid_y:
                v_new.append(new_vel)
            else:
                if not valid_x:
                    new_vel[0] = 0  
                if not valid_y:
                    new_vel[1] = 0  

                if inner_path.contains_point([curr_pos[0] + new_vel[0], curr_pos[1] + new_vel[1]]) and \
                    outer_path.contains_point([curr_pos[0] + new_vel[0], curr_pos[1] + new_vel[1]]):
                        v_new.append(new_vel)

        
        v_new = np.array(v_new)
        grad = min(v_new, key=lambda v: self.cost_function1(v, vel_d))

        if np.abs(yc) > self.threshold:
            grad = np.zeros(2)
        # print(f'{agent_x:.2f}, {agent_y:.2f}, {yrel:.2f}, {yc:.2f}, {grad[0]:.2f}, {grad[1]:.2f}')
        return grad
    
    def cost_function1(self, v, vel_d):
        return np.linalg.norm(np.array(v) - np.array(vel_d))
   
    def pad_boundaries(self, boundary_points, thickness):
        original_polygon = ShapelyPolygon(boundary_points)
        outer_polygon = original_polygon.buffer(thickness)
        inner_polygon = original_polygon.buffer(-thickness)

        outer_boundary = list(LinearRing(outer_polygon.exterior).coords)
        inner_boundary = list(LinearRing(inner_polygon.exterior).coords)

        return np.array(outer_boundary), np.array(inner_boundary)
