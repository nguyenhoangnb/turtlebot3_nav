#!/usr/bin/env python3

from geometry_msgs.msg import Point32, Pose2D, Twist
from std_msgs.msg import String
import rclpy
import numpy as np
import math
from typing import List, Tuple
import copy

class Jacobians:
    def __init__(self):
        self.Zp = np.zeros(2)
        self.Hv = np.zeros((2, 3))
        self.Hf = np.zeros((2, 2))
        self.Sf = np.zeros((2, 2))

class Particle:
    def __init__(self, n_particles):
        self.n_particles = n_particles
        self.w = 1.0 / n_particles
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # Fixed: Proper 2D array initialization
        self.P = np.array([
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.1]
        ])
        self.landmarks = []
    
    def predict(self, u, dt):
        self.x = self.x + u[0] * dt
        self.y = self.y + u[1] * dt
        self.yaw = self.yaw + u[2] * dt
        self.yaw = normalize_angle(self.yaw)

    def compute_weight(self, id, z, Q):
        xf = self.landmarks[id].x
        Pf = self.landmarks[id].P

        J = compute_jacobians(self, xf, Pf, Q)
        dz = np.array([
            z[0] - J.Zp[0], 
            normalize_angle(z[1] - J.Zp[1])  # Fixed: J.Zp instead of J.zp
        ])

        try:
            invS = np.linalg.inv(J.Sf)
        except np.linalg.LinAlgError:
            return 1.0
        
        num = np.exp(-0.5 * dz.T @ invS @ dz)
        den = 2.0 * np.pi * np.sqrt(np.linalg.det(J.Sf))

        if np.isinf(num) or den == 0:
            return 1.0
        
        weight = num / den
        return weight

    def proposal_sampling(self, id, z, Q):
        xf = self.landmarks[id].x
        Pf = self.landmarks[id].P  # Fixed: .P instead of .pop
        J = compute_jacobians(self, xf, Pf, Q)
        
        try:
            Sinv = np.linalg.inv(J.Sf)
        except np.linalg.LinAlgError:
            return  # Skip if matrix is singular

        dz = np.array([
            z[0] - J.Zp[0],
            normalize_angle(z[1] - J.Zp[1])
        ])

        try:
            Pi = np.linalg.inv(self.P)
            self.P = np.linalg.inv(J.Hv.T @ Sinv @ J.Hv + Pi)
            dx = self.P @ J.Hv.T @ Sinv @ dz

            self.x += dx[0]
            self.y += dx[1]
            self.yaw += dx[2]
            self.yaw = normalize_angle(self.yaw)
        except np.linalg.LinAlgError:
            pass  # Skip update if matrix operations fail

class Landmark:
    def __init__(self, z, Q, particle: Particle):
        r = z[0]
        b = z[1]
        c = math.cos(normalize_angle(particle.yaw + b))
        s = math.sin(normalize_angle(particle.yaw + b))
        
        # Fixed: Proper array initialization
        self.x = np.array([
            particle.x + r * c,
            particle.y + r * s
        ])
        
        dx = r * c
        dy = r * s
        d2 = dx * dx + dy * dy
        d = math.sqrt(d2)
        
        # Fixed: Proper 2D array
        Gz = np.array([
            [dx / d, dy / d],
            [-dy / d2, dx / d2]
        ])
        
        try:
            Gz_inv = np.linalg.inv(Gz)
            self.P = Gz_inv @ Q @ Gz_inv.T
        except np.linalg.LinAlgError:
            self.P = np.eye(2) * 0.1  # Default covariance

        self.n_observed = 0  

    def update(self, z, Q, particle: Particle):
        J = compute_jacobians(particle, self.x, self.P, Q)  # Fixed: self.P instead of self.p
        dz = np.array([
            z[0] - J.Zp[0],  # Fixed: J.Zp instead of J @ z[0]
            normalize_angle(z[1] - J.Zp[1])
        ])

        try:
            Pht = self.P @ J.Hf.T
            Sf = J.Hf @ Pht + Q
            Sfa = 0.5 * (Sf + Sf.T)
            
            # Use Cholesky decomposition safely
            try:
                L = np.linalg.cholesky(Sfa)
                Schol = L.T
                Sinv = np.linalg.inv(Schol)
            except np.linalg.LinAlgError:
                # Fallback to regular inverse
                Sinv = np.linalg.inv(Sf)
                
            W1 = Pht @ Sinv
            W = W1 @ Sinv.T
            
            self.x = self.x + W @ dz
            self.P = self.P - W1 @ W1.T
            
            # Ensure P remains positive definite
            eigenvals, eigenvecs = np.linalg.eigh(self.P)
            eigenvals = np.maximum(eigenvals, 1e-6)
            self.P = eigenvecs @ np.diag(eigenvals) @ eigenvecs.T
            
        except np.linalg.LinAlgError:
            pass  # Skip update if matrix operations fail

        self.n_observed += 1

    def pdf(self, z):
        try:
            n = z.shape[0]
            sqrt2pi = np.sqrt(2 * math.pi)

            diff = z - self.x
            P_inv = np.linalg.inv(self.P)
            quadform = diff.T @ P_inv @ diff
            norm = np.power(sqrt2pi, -n) * np.power(np.linalg.det(self.P), -0.5)
            return norm * np.exp(-0.5 * quadform)
        except:
            return 1e-10

    def mahalanobis(self, z):
        try:
            diff = self.x - z
            return np.sqrt(diff.T @ np.linalg.inv(self.P) @ diff)
        except:
            return np.inf

class Slam:
    def __init__(self):
        self.landmarks_map_ = []
        self.particles_: List['Particle'] = []
        self.landmark_map_: List[Tuple[float, float]] = []
        self.slam_state_ = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0
        }
        self.max_map_size_ = 0
        self.n_particles_ = 0
        self.n_resample_ = 0  # Fixed: n_resample_ instead of n_samples_
        self.best_id_ = 0  # Fixed: best_id_ instead of best_id

    def get_map(self) -> List[Point32]:
        return self.landmarks_map_

    def get_state(self) -> dict:
        return self.slam_state_

    def set_parameters(self, n_particles: int):  # Fixed: set_parameters instead of set_parameter
        self.n_particles_ = n_particles
        self.n_resample_ = int(n_particles / 1.5)  # Fixed: n_resample_
        self.particles_ = [Particle(n_particles) for _ in range(n_particles)]
        
    def initialize_state(self):
        self.slam_state_['x'] = 0.0
        self.slam_state_['y'] = 0.0
        self.slam_state_['theta'] = 0.0

    def predict_particles(self, u: np.ndarray, R: np.ndarray, dt: float):
        for particle in self.particles_:
            noise = np.random.normal(0.0, 1.0, size=(3,))
            
            # Fixed: Proper noise calculation
            Rsqrt = np.sqrt(np.abs(np.diag(R)))
            ud = u + (noise * Rsqrt)
            particle.predict(ud, dt)

    def normalize_weight(self):
        sum_w = sum([p.w for p in self.particles_])
        if sum_w == 0 or not np.isfinite(sum_w):  # Fixed: not np.isfinite
            for p in self.particles_:
                p.w = 1.0 / max(1, self.n_particles_)
        else:
            for p in self.particles_:
                p.w = p.w / sum_w

    def resample(self):
        self.normalize_weight()
        weight = np.array([p.w for p in self.particles_])
        n_eff = 1.0 / np.sum(weight * weight) if weight.size > 0 else 0.0
        
        if n_eff < self.n_resample_:
            prob = weight.copy()
            s = prob.sum()
            if s <= 0 or not np.isfinite(s):
                prob = np.ones(len(self.particles_)) / len(self.particles_)
            else:
                prob = prob / s
            
            try:
                idxs = np.random.choice(len(self.particles_), size=len(self.particles_), replace=True, p=prob)
                resamp = [copy.deepcopy(self.particles_[i]) for i in idxs]
                for p in resamp:
                    p.w = 1.0 / len(resamp)
                self.particles_ = resamp
            except:
                # Fallback: reset all weights
                for p in self.particles_:
                    p.w = 1.0 / len(self.particles_)

    def update_landmarks(self, landmarks: List[Tuple[float, float]], frozen_update: bool = False):
        # particle_lm: which landmark indices each particle observed this update
        particle_lm = [[] for _ in range(self.n_particles_)]

        for i, lm_pt in enumerate(landmarks):
            # accept either tuple/list or object with x,y
            lx, ly = (lm_pt.x, lm_pt.y) if hasattr(lm_pt, 'x') else (lm_pt[0], lm_pt[1])

            for pid, particle in enumerate(self.particles_):
                s = np.sin(particle.yaw)
                c = np.cos(particle.yaw)

                # Transform from robot frame (measurement) to world frame
                z_world_x = (lx * c - ly * s) + particle.x
                z_world_y = (lx * s + ly * c) + particle.y

                # measurement in polar (range,bearing) relative to robot pose
                z = np.array([
                    np.hypot(lx, ly),
                    normalize_angle(np.arctan2(ly, lx))
                ])

                Q = np.array([[0.001, 0.0],
                              [0.0,   0.0025]])

                # find nearest landmark in particle's map
                if len(particle.landmarks) == 0:
                    nearest_idx = None
                    nearest_dist = np.inf
                else:
                    dists = [np.hypot(lm.x[0] - z_world_x, lm.x[1] - z_world_y) for lm in particle.landmarks]
                    nearest_idx = int(np.argmin(dists))
                    nearest_dist = dists[nearest_idx]

                if nearest_idx is None:
                    # add new landmark
                    new_lm = Landmark(z, Q, particle)
                    particle.landmarks.append(new_lm)
                    observed_idx = len(particle.landmarks) - 1
                else:
                    # decide whether to add or update
                    if nearest_dist > 0.5 and (not frozen_update):
                        # add new landmark
                        new_lm = Landmark(z, Q, particle)
                        particle.landmarks.append(new_lm)
                        observed_idx = len(particle.landmarks) - 1
                    else:
                        # update existing landmark
                        p_idx = nearest_idx
                        # update particle weight by measurement likelihood
                        particle.w *= particle.compute_weight(p_idx, z, Q)  # Fixed: compute_weight
                        # update landmark via Kalman-like update
                        particle.landmarks[p_idx].update(z, Q, particle)
                        # update particle state via proposal sampling step
                        particle.proposal_sampling(p_idx, z, Q)  # Fixed: proposal_sampling
                        observed_idx = p_idx

                particle_lm[pid].append(observed_idx)

        # prune landmarks per particle: keep observed ones or those with n_observed > 5
        for i, particle in enumerate(self.particles_):
            new_list = []
            for j, lm in enumerate(particle.landmarks):
                if (j in particle_lm[i]) or (lm.n_observed > 5):
                    new_list.append(lm)
            particle.landmarks = new_list

    def calc_final_state(self):
        self.normalize_weight()
        sx = 0.0
        sy = 0.0
        st = 0.0
        for p in self.particles_:
            sx += p.x * p.w
            sy += p.y * p.w
            st += p.yaw * p.w
        self.slam_state_['x'] = sx
        self.slam_state_['y'] = sy
        self.slam_state_['theta'] = normalize_angle(st)

    def create_map(self):
        self.landmarks_map_.clear()
        max_w = -np.inf
        best_id = 0
        for idx, p in enumerate(self.particles_):
            if p.w > max_w:
                max_w = p.w
                best_id = idx
        self.best_id_ = best_id

        best_particle = self.particles_[self.best_id_]
        self.landmarks_map_ = []
        for lm in best_particle.landmarks:
            self.landmarks_map_.append((float(lm.x[0]), float(lm.x[1])))

    def calculate_state(self, velocity):
        if hasattr(velocity, 'linear'):
            self.slam_state_['x'] += velocity.linear.x
            self.slam_state_['y'] += velocity.linear.y
            self.slam_state_['theta'] += velocity.angular.z
        elif isinstance(velocity, dict):
            self.slam_state_['x'] += velocity.get('linear_x', 0.0)
            self.slam_state_['y'] += velocity.get('linear_y', 0.0)
            self.slam_state_['theta'] += velocity.get('angular_z', 0.0)
        else:
            # expect tuple (vx, vy, vyaw)
            try:
                vx, vy, vw = velocity
                self.slam_state_['x'] += vx
                self.slam_state_['y'] += vy
                self.slam_state_['theta'] += vw
            except Exception:
                raise ValueError("Unsupported velocity type")
        
        self.slam_state_['theta'] = normalize_angle(self.slam_state_['theta'])

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def compute_jacobians(particle: Particle, xf, Pf, Q_cov):
    j = Jacobians()
    
    dx = xf[0] - particle.x
    dy = xf[1] - particle.y
    d2 = dx**2 + dy**2
    distance = math.sqrt(d2)
    
    # Fixed: Proper assignment
    j.Zp[0] = distance
    j.Zp[1] = normalize_angle(math.atan2(dy, dx) - particle.yaw)

    j.Hv[0, :] = [-dx/distance, -dy/distance, 0.0]
    j.Hv[1, :] = [dy/d2, -dx/d2, -1.0]  # Fixed: -1.0 instead of -1/0

    j.Hf[0, :] = [dx/distance, dy/distance]  # Fixed: /distance not /d2
    j.Hf[1, :] = [-dy/d2, dx/d2]

    j.Sf = j.Hf @ Pf @ j.Hf.T + Q_cov

    return j