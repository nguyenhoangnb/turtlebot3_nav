from dataclasses import dataclass
import math
from rigid2D import *

@dataclass
class WheelAngle:
    w_ang1: float = 0.0
    w_ang2: float = 0.0

@dataclass
class WheelVel:
    w1_vel: float = 0.0
    w2_vel: float = 0.0

@dataclass 
class Configuration:
    x_config: float = 0.0
    y_config: float = 0.0
    theta_config: float = 0.0

class DiffDrive:
    def __init__(self, q:Configuration = Configuration(), r: float=0.033, d: float=0.16):
        self.w_ang = WheelAngle(0.0, 0.0)
        self.w_vel = WheelVel(0.0, 0.0)
        self.q = q
        self.r = r
        self.d = d
    
    def forward_kinematic(self, new_wheel_angle)->Configuration:
        pass

    def inverse_kinematic(self, V: Twist2D)->WheelVel:
        pass