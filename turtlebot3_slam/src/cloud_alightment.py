#!/usr/bin/env python3

import numpy as np 
import open3d as o3d
import math
from typing import List, Tuple
from dataclasses import dataclass
from sensor_properties import *

    
class ScanAlightment:
    def __init__(self, props: LaserProperties, Trs: Transform2D):
        self.max_iter = 100
        self.max_correspondence_dis = 0.5
        self.transform_epsilon = 1e-8
        self.fitness_epsilon = 1e-6
        self.Trs = Trs
        self.beam_max = props.beam_max
        self.beam_min = props.beam_min
        self.beam_delta = props.beam_delta
        self.range_max = props.range_max
        self.range_min = props.range_min
        self.first_scan_receive = False
        self.old_scan = []
    
    def create_point_cloud(self, beam_lenght):
        points = []
        beam_angle = self.beam_min
        for r in beam_lenght:
            if self.range_min <= r < self.range_max:
                px = r * np.cos(beam_angle)
                py = r * np.sin(beam_angle)

                px, py = self.Trs((px, py))
                points.append([px, py, 0.0])
            
            beam_angle += self.beam_delta
        
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.array(points))
        return cloud
    
    def pcl_icp_wrapper(self, T: Transform2D, T_init, beam_length):
        if self.first_scan_receive:
            target_cloud = self.create_point_cloud(self.old_scan)
            source_cloud = self.create_point_cloud(beam_length)
            success, T_new = self.pcl_icp(T_init, target_cloud, source_cloud)
            if not success:
                return False
            
            T.x, T.y, T.theta = T_new.x, T_new.y, T_new.theta
        
        else:
            self.old_scan = beam_length[:]
            self.first_scan_receive = True
        
        return True
    
    def pcl_icp(self, T_init: Transform2D, target, source):
        threshold = self.max_correspondence_dis
        init_guess = T_init.to_matrix()
        
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, init_guess, 
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(  # Fixed typo
                max_iteration=self.max_iter
            )  
        )
        if reg_p2p.fitness < 0.8 or reg_p2p.inlier_rmse > 0.1:  
            return False, Transform2D()
        
        T_mat = reg_p2p.transformation
        theta = np.arctan2(T_mat[1, 0], T_mat[0, 0])  # Fixed index
        tx, ty = T_mat[0, 3], T_mat[1, 3]
        return True, Transform2D(tx, ty, theta)  # Fixed constructor call



