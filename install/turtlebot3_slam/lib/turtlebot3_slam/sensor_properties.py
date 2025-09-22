from dataclasses import dataclass
import math
from typing import List, Tuple
import numpy as np 
from rigid2D import *
def normalize_angle_pi(angle: float)->float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2*math.pi
    
    return angle

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
        Tms = pose * self.Trs
        beam_angle = self.beam_min
        for r in beam_length:
            if self.range_min <= r <= self.range_max:
                xs = r * math.cos(beam_angle)
                ys = r * math.sin(beam_angle)

                p_map = Tms(Vector2D(xs, ys))
                if isinstance(p_map, Vector2D):
                    end_points.append(p_map)
                else:
                    end_points.append(Vector2D(p_map[0], p_map[1]))
            
            beam_angle += self.beam_delta

            # Fix beam angle wrapping logic
            if self.beam_max < 0.0 and beam_angle <= self.beam_max:
                break
            elif self.beam_max >= 0.0 and beam_angle >= self.beam_max:
                break
                
        return end_points
    
    def number_valid_measurements(self, beam_length: List[float])->int:
        return sum (1 for r in beam_length if self.range_min <= r <= self.range_max)
