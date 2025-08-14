from dataclasses import dataclass
import math
from typing import List, Tuple
import numpy as np 
def normalize_angle_pi(angle: float)->float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    
    return angle

@dataclass
class Vector2D:
    x: float = 0.0
    y: float = 0.0

@dataclass
class LaserProperties:
    beam_min: float = 0.0
    beam_max: float = 0.0
    beam_delta: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    z_hit: float = 0.25
    z_short: float = 0.25
    z_max: float = 0.25
    z_rand: float = 0.25 
    sigma_hit: float = 1.0


class Transform2D:
    def __init__(self, 
                 x=0.0,
                 y=0.0,
                 theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def to_matrix(self):
        c, s = np.cos(self.theta), np.sin(self.theta)
        return np.array([
            [c, -s, 0.0, self.x],
            [s, c, 0.0, self.y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)
    
    def __call__(self, point):
        px, py = point
        c, s = np.cos(self.theta), np.sin(self.theta)
        x_new = c*px - s*py + self.x
        y_new = s*px + c*py + self.y
        return np.array([x_new, y_new])
    
    def compose(self, other):
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        x_new = self.x + c*other.x - s * other.y
        y_new = self.y + s * other.x - c * other.y
        theta_new = normalize_angle_pi(self.theta + other.theta)
        return Transform2D(x_new, y_new, theta_new)
    
    def inverse(self):
        theta_inv = normalize_angle_pi(-self.theta)
        c = math.cos(theta_inv)
        s = math.sin(theta_inv)

        x_inv = -(c*self.x - s*self.y)
        y_inv = -(s*self.x + c*self.y)
        return Transform2D(x_inv, y_inv, theta_inv)
    
class LaserScanner:
    def __init__(self, props: LaserProperties, Trs: Transform2D):
        self.z_hit = props.z_hit
        self.z_short = props.z_short
        self.beam_min = props.beam_min
        self.beam_max = props.beam_max
        self.beam_delta = props.beam_delta
        self.range_min = props.range_min
        self.range_max = props.range_max
        self.z_max = props.z_max
        self.z_rand = props.z_rand 
        self.sigma_hit = props.sigma_hit

        self.Trs = Trs
    
    def laser_end_points(self, beam_length: List[float],
                       pose: Transform2D) -> List[Vector2D]:
        end_points = []
        angle = self.beam_min
        for r in beam_length:
            if self.range_min <= r <= self.range_max:
                xs = r * math.cos(angle)
                ys = r * math.sin(angle)

                p_robot = self.Trs((xs, ys))

                p_map = pose(p_robot)
                end_points.append(Vector2D(p_map[0], p_map[1]))
            
            angle += self.beam_delta
        
        return end_points
    
    def number_valid_measurements(self, beam_length: List[float])->int:
        return sum (1 for r in beam_length if self.range_min <= r <= self.range_max)
