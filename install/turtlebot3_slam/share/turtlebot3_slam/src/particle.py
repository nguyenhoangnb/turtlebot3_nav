#!/usr/bin/env python3
import numpy as np 
import math 
import random 
from typing import List, Tuple, Optional
from dataclasses import dataclass
from copy import deepcopy

from sensor_properties import *
from grid_mapper import *
from cloud_alignment import *
from rigid2D import *
def normalize_angle_pi(angle: float)->float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    
    return angle

def almost_equal(a: float, b: float, tolerance: float = 1e-9)->bool:
    return abs(a - b) < tolerance

def pdf_normal(x: float, var: float) -> float:
    """Probability density function for normal distribution"""
    if abs(var) < 1e-12:
        return 0.0
    
    # Prevent overflow in exponential
    exp_arg = -0.5 * x * x / var
    if exp_arg < -700:  # exp(-700) is close to 0
        return 0.0
    
    try:
        return (1.0 / math.sqrt(2 * math.pi * var)) * math.exp(exp_arg)
    except (OverflowError, ValueError):
        return 0.0

# @dataclass
# class Twist2D:
#     vx: float = 0.0
#     w: float = 0.0

@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

@dataclass
class Twist2DData:
    vx: float = 0.0
    w: float = 0.0  # Changed from theta to w for consistency
@dataclass
class Particle:
    weight: float
    grid: GridMapper
    pose: np.ndarray
    prev_pose: np.ndarray

    def __init__(self, weight: float, mapper: GridMapper, pose: np.ndarray):
        self.weight = weight
        self.grid = deepcopy(mapper)
        self.pose = pose.copy()
        self.prev_pose = pose.copy()

class ParticleFilter:

    def __init__(self,
                 num_particles: int, 
                 k: int,
                 srr: float,
                 srt: float,
                 str_: float,
                 stt: float,
                 motion_noise_theta: float,
                 motion_noise_x: float,
                 motion_noise_y: float,
                 sample_range_theta: float,
                 sample_range_x: float,
                 sample_range_y: float,
                 scan_likelihood_min: float,
                 scan_likelihood_max: float,
                 pose_likelihood_min: float,
                 pose_likelihood_max: float,
                 scan_matcher: ScanAlightment,
                 pose: Transform2D,
                 mapper: GridMapper):
        self.num_particles_ =  num_particles
        self.k_ = k
        self.srr_ = srr
        self.srt_ = srt
        self.str_ = str_
        self.stt_ = stt
        self.motion_noise_theta_ = motion_noise_theta
        self.motion_noise_x_ = motion_noise_x
        self.motion_noise_y_ = motion_noise_y
        self.sample_range_theta_ = sample_range_theta
        self.sample_range_x_ = sample_range_x
        self.sample_range_y_ = sample_range_y
        self.scan_likelihood_min_ = scan_likelihood_min
        self.scan_likelihood_max_ = scan_likelihood_max
        self.pose_likelihood_min_ = pose_likelihood_min
        self.pose_likelihood_max_ = pose_likelihood_max
        self.scan_matcher_ = scan_matcher
        
        self.normal_sqrd_sum_ = 0.0
        self.motion_noise_ = np.diag([motion_noise_theta, motion_noise_x, motion_noise_y])
        self.sample_range_ = np.diag([sample_range_theta, sample_range_x, sample_range_y])

        self.init_particle_set(mapper, pose)
    
    def init_particle_set(self, mapper: GridMapper, pose: Transform2D) -> None:
        weight = 1.0 / self.num_particles_
        self.particle_set_: List[Particle] = []
        for i in range(self.num_particles_):
            pose_array = np.array([pose.theta, pose.v2.x, pose.v2.y])
            particle = Particle(weight, mapper, pose_array)
            self.particle_set_.append(particle)
    
    def sample_multivariate_normal(self, mean: np.ndarray, cov: np.ndarray)-> np.ndarray:
        try:
            # Check for valid covariance matrix
            if np.any(np.diag(cov) <= 0):
                return mean.copy()
            
            # Check for reasonable values to prevent overflow
            max_std = np.sqrt(np.max(np.diag(cov)))
            if max_std > 100:  # Prevent extreme sampling
                # Scale down the covariance
                cov = cov * (100.0 / max_std) ** 2
                
            return np.random.multivariate_normal(mean, cov)
        except (np.linalg.LinAlgError, ValueError, OverflowError):
            # Fallback to mean if sampling fails
            return mean.copy()
    
    def sample_standard_normal(self, n: int)->np.ndarray:
        return np.random.randn(n)
    
    def SLAM(self, scan: List[float], u: Twist2DData, cur_odom: Pose, prev_odom: Pose):
        cur_od = np.array([cur_odom.theta, cur_odom.x, cur_odom.y])
        prev_od = np.array([prev_odom.theta, prev_odom.x, prev_odom.y])

        T_init = self.icp_init_guess(cur_od, prev_od)

        T_icp = Transform2D()
        matcher_success = self.scan_matcher_.pcl_icp_wrapper(T_icp, T_init, scan)
        for particle in self.particle_set_:
            if not matcher_success:
                particle.prev_pose = particle.pose.copy()
                self.sample_motion_model(u, particle.pose)

                T_pose = Transform2D(Vector2D(particle.pose[1], particle.pose[2]), particle.pose[0]) 

                scan_likelihood = particle.grid.likelihood_field_model(scan, T_pose)
                particle.weight *= scan_likelihood
            else:
                vt = Vector2D(particle.pose[1], particle.pose[2])
                T_x = Transform2D(vt, particle.pose[0])
                T_icp_inv = T_icp.inv()
                T_x = self.compose_transforms(T_x, T_icp_inv)

                sample_poses = self.sample_mode(T_x)
                mu = np.zeros(3)
                sigma = np.zeros((3, 3))
                eta = 0.0

                mu, sigma, eta = self.gaussian_proposal(
                    sample_poses, particle, scan, cur_od, prev_od
                )

                if eta > 1e-12:
                    new_pose = self.sample_multivariate_normal(mu, sigma)
                    new_pose[0] = normalize_angle_pi(new_pose[0])

                    particle.prev_pose = particle.pose.copy()
                    particle.pose = new_pose
                    
                    particle.weight *= eta
                else:
                    # Fallback to motion model if proposal failed
                    particle.prev_pose = particle.pose.copy()
                    self.sample_motion_model(u, particle.pose)
                    
                    T_pose = Transform2D(Vector2D(particle.pose[1], particle.pose[2]), particle.pose[0])
                    scan_likelihood = particle.grid.likelihood_field_model(scan, T_pose)
                    particle.weight *= scan_likelihood
            vt = Vector2D(particle.pose[1], particle.pose[2])
            T_particle = Transform2D(vt, particle.pose[0])
            particle.grid.integrate_scan(scan, T_particle)
        self.normalize_weights()
        if self.effective_particles():
            print("Resampling")
            self.low_variance_resampling()
    
    def sample_motion_model(self, u: Twist2DData, pose: np.ndarray)->None:
        try:
            w = self.sample_multivariate_normal(np.zeros(3), self.motion_noise_)
            
            # Clamp extreme values
            w = np.clip(w, -10.0, 10.0)
            
            if almost_equal(u.w, 0.0):
                pose[0] = normalize_angle_pi(pose[0] + w[0])
                pose[1] += u.vx * math.cos(pose[0]) + w[1]
                pose[2] += u.vx * math.sin(pose[0]) + w[2]
            else:
                old_theta = pose[0]
                pose[0] = normalize_angle_pi(pose[0] + u.w + w[0])
                
                # Prevent division by very small numbers
                w_safe = u.w if abs(u.w) > 1e-6 else 1e-6 * np.sign(u.w) if u.w != 0 else 1e-6
                
                pose[1] += (-u.vx/w_safe) * math.sin(old_theta) +\
                           (u.vx / w_safe) * math.sin(old_theta + u.w) + w[1]
                
                pose[2] += (u.vx/w_safe) * math.cos(old_theta) -\
                           (u.vx / w_safe) * math.cos(old_theta + u.w) + w[2]
                           
            # Clamp pose to reasonable bounds
            pose[1] = np.clip(pose[1], -100.0, 100.0)
            pose[2] = np.clip(pose[2], -100.0, 100.0)
            
        except (OverflowError, ValueError):
            # Add minimal noise if calculation fails
            pose[0] = normalize_angle_pi(pose[0] + np.random.normal(0, 0.01))
            pose[1] += np.random.normal(0, 0.01)
            pose[2] += np.random.normal(0, 0.01)

    def pose_likelihood_odom(self, cur_pose: np.ndarray,
                                   prev_pose: np.ndarray,
                                   cur_odom: np.ndarray,
                                   prev_odom: np.ndarray)-> float:
        rot1 = math.atan2(cur_odom[2] - prev_odom[2], 
                         cur_odom[1] - prev_odom[1]) - prev_odom[0]
        trans = math.sqrt((cur_odom[1] - prev_odom[1])**2 + 
                          (cur_odom[2] - prev_odom[2])**2)
        
        rot2 = normalize_angle_pi(normalize_angle_pi(cur_odom[0]) - 
                                  normalize_angle_pi(prev_odom[0]) - rot1)
        
        rot1_hat = math.atan2(cur_pose[2] - prev_pose[2], 
                              cur_pose[1] - prev_pose[1]) - prev_pose[0]
        
        trans_hat = math.sqrt((cur_pose[1] - prev_pose[1])**2 + 
                              (cur_pose[2] - prev_pose[2])**2)
        
        rot2_hat = normalize_angle_pi(normalize_angle_pi(cur_pose[0]) - 
                                      normalize_angle_pi(prev_pose[0]) - rot1_hat)
        
        temp1 = self.srr_ * rot1_hat ** 2 + self.srt_ * trans_hat**2
        temp2 = self.str_ * trans_hat**2 + self.stt_ * rot1_hat**2 + self.stt_ * rot2_hat**2
        temp3 = self.srr_ * rot2_hat**2 + self.srt_ * trans_hat**2

        p1 = pdf_normal(normalize_angle_pi(rot1 - rot1_hat), temp1)
        p2 = pdf_normal(trans - trans_hat, temp2)
        p3 = pdf_normal(normalize_angle_pi(rot2 - rot2_hat) , temp3)

        return p1 * p2 * p3
    
    def normalize_weights(self) -> None:
        # Extract weights for processing
        weights = np.array([p.weight for p in self.particle_set_])
        
        # Check for weight overflow or NaN
        if np.any(np.isnan(weights)) or np.any(np.isinf(weights)) or np.all(weights == 0):
            print("Weight overflow detected, resetting to uniform")
            weights = np.ones(len(self.particle_set_)) / len(self.particle_set_)
        else:
            # Normalize weights with stability check
            weight_sum = np.sum(weights)
            if weight_sum <= 1e-50:  # Tăng threshold
                weights = np.ones(len(self.particle_set_)) / len(self.particle_set_)
            else:
                weights = weights / weight_sum
                
        # Apply exponential smoothing to weights để ổn định hơn
        if hasattr(self, 'prev_weights') and len(self.prev_weights) == len(weights):
            alpha = 0.7  # Smoothing factor
            weights = alpha * weights + (1 - alpha) * self.prev_weights
            weights = weights / np.sum(weights)  # Re-normalize
        
        self.prev_weights = weights.copy()  # Store for next iteration
        
        # Update particle weights
        for i, particle in enumerate(self.particle_set_):
            particle.weight = weights[i]
            
        # Calculate normalized squared sum for effective sample size
        self.normal_sqrd_sum_ = np.sum(weights ** 2)
    
    def effective_particles(self)->bool:
        n_eff = 1.0 / self.normal_sqrd_sum_ if self.normal_sqrd_sum_ > 0 else 0
        # Tăng threshold để resampling ít hơn, giữ ổn định hơn
        return int(n_eff) < (self.num_particles_ * 0.6)  # Tăng từ //2 lên 0.6
    
    def low_variance_resampling(self)->None:
        temp_particles = []
        r = np.random.random() / self.num_particles_
        c = self.particle_set_[0].weight
        i = 0
        for m in range(self.num_particles_):
            u = r + m / self.num_particles_
            while u > c:
                i += 1
                if i < len(self.particle_set_):
                    c += self.particle_set_[i].weight
                else:
                    break
            
            if i < len(self.particle_set_):
                new_particle = Particle(
                    1.0/self.num_particles_,
                    self.particle_set_[i].grid,
                    self.particle_set_[i].pose
                )
                new_particle.prev_pose = self.particle_set_[i].prev_pose.copy()
                temp_particles.append(new_particle)
        
        self.particle_set_ = temp_particles
    
    def sample_mode(self, T: Transform2D)-> List[np.ndarray]:
        sample_poses = []
        mu = np.array([T.theta, T.v2.x, T.v2.y])  # Fixed: use v2.x, v2.y

        for _ in range(self.k_):
            sample = self.sample_multivariate_normal(mu, self.sample_range_)
            sample[0] = normalize_angle_pi(sample[0])
            sample_poses.append(sample)
        
        return sample_poses
    
    def gaussian_proposal(self, sampled_poses: List[np.ndarray], 
                                particle: Particle, 
                                scan: List[float],
                                cur_odom: np.ndarray,
                                pre_odom: np.ndarray)->Tuple[np.ndarray, np.ndarray, float]:

        likelihoods = []
        mu = np.zeros(3)
        sigma = np.zeros((3, 3))
        eta = 0.0

        for pose in sampled_poses:
            T_pose = Transform2D(Vector2D(pose[1], pose[2]), pose[0])  # Fixed constructor

            p_scan = particle.grid.likelihood_field_model(scan, T_pose)
            p_pose = self.pose_likelihood_odom(pose, particle.prev_pose, cur_odom, pre_odom)

            p_scan = max(min(p_scan, self.scan_likelihood_max_), self.scan_likelihood_min_)
            p_pose = max(min(p_pose, self.pose_likelihood_max_), self.pose_likelihood_min_)

            p = p_scan * p_pose

            likelihoods.append(p)
            mu += pose * p
            eta += p
        if eta > 1e-12:
            mu /= eta
            mu[0] = normalize_angle_pi(mu[0])

            for i, pose in enumerate(sampled_poses):
                diff = pose - mu
                sigma += np.outer(diff, diff) * likelihoods[i]
            
            sigma /= eta

        return mu, sigma, eta
    
    def icp_init_guess(self, cur_odom: np.ndarray, prev_odom: np.ndarray)->Transform2D:
        dx = cur_odom[1] - prev_odom[1]
        dy = cur_odom[2] - prev_odom[2]
        dth = normalize_angle_pi(normalize_angle_pi(cur_odom[0]) - normalize_angle_pi(prev_odom[0]))
        c = math.cos(prev_odom[0])
        s = math.sin(prev_odom[0]) 
        dx_rot = c * dx + s * dy
        dy_rot = -s * dx + c * dy

        return Transform2D(Vector2D(dx_rot, dy_rot), dth)  
    
    def compose_transforms(self, T1: Transform2D, T2: Transform2D)->Transform2D:
        return T1.compose(T2)

    def get_robot_state(self)->Transform2D:
        best_particle = max(self.particle_set_, key=lambda p: p.weight)
        pose = best_particle.pose
        return Transform2D(Vector2D(pose[1], pose[2]), pose[0])  
    
    def new_map(self)-> List[int]:
        best_particle = max(self.particle_set_, key=lambda p: p.weight)
        return best_particle.grid.grid_map()