# -*- coding: utf-8 -*-
"""
Created on Thu Dec  3 22:13:49 2020

@author: Kunal Nalamwar
"""
#This script is used for post processing of the acquired map points for active, dynamic and the ground truth
import numpy as np
import pandas as pd

#%% This part of the code is just for loading data and creating arrays
#Instead of writing this part in a smart way, I am just loading all the csv files in different arrays 
df = pd.read_csv('Ground_truth.csv', index_col=None, header=0, skiprows=0, usecols=[0,1,2])
ground_truth_map_points = df.to_numpy()
print("The shape of ground truth matrix: ", np.shape(ground_truth_map_points))
df1 = pd.read_csv('Dynamic_points.csv', index_col=None, header=0, skiprows=0, usecols=[0,1,2])
dynamic_map_points = df1.to_numpy()
print("The shape of Dynamic points matrix: ", np.shape(dynamic_map_points))
df2 = pd.read_csv('Static_points.csv', index_col=None, header=0, skiprows=0, usecols=[0,1,2])
static_map_points = df2.to_numpy()
print("The shape of Active points matrix: ", np.shape(static_map_points))

#%% This part of the code is used for processing the data arrays created earlier
matched_points_dynamic = 0
for i in range(0, np.shape(ground_truth_map_points)[0]):
    if (ground_truth_map_points[i,0] == dynamic_map_points[i,0] and ground_truth_map_points[i,1] == dynamic_map_points[i,1] and ground_truth_map_points[i,2] == dynamic_map_points[i,2] ):
        matched_points_dynamic = matched_points_dynamic + 1
        
print("number of matched dynamic points : ", matched_points_dynamic)

matched_points_static = 0
for i in range(0, np.shape(ground_truth_map_points)[0]):
    if (ground_truth_map_points[i,0] == static_map_points[i,0] and ground_truth_map_points[i,1] == static_map_points[i,1] and ground_truth_map_points[i,2] == static_map_points[i,2] ):
        matched_points_static = matched_points_static + 1
        
print("number of matched static points : ", matched_points_static)