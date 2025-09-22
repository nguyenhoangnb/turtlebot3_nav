from dataclasses import dataclass, field
import math
import heapq
from typing import Tuple, List, Optional, Iterable
import numpy as np
from sensor_properties import *
PI = math.pi

def log_odds_2_prob(l: float)-> float:
    return 1 / (1 + math.exp(-l))

def prob_2_log_odds(p: float) -> float:
    return math.log(p / (1.0 - p))

def pdf_normal(a: float, b: float)->float:
    if (abs(b) < 1e-12):
        return 0.0
    
    # Prevent overflow in exponential
    exp_arg = -0.5 * (a * a)/b
    if exp_arg < -700:  # exp(-700) is close to 0
        return 0.0
    
    try:
        sqrt_inv = 1.0 / math.sqrt(2.0 * math.pi * b)
        return sqrt_inv * math.exp(exp_arg)
    except (OverflowError, ValueError):
        return 0.0

def map_size(lower: float, upper: float, resolution: float)->int:
    return int(math.ceil((upper - lower) / resolution))

@dataclass
class GridCoordinates:
    i: int = 0
    j: int = 0

@dataclass
class Cell:
    log_odds: float = 0.0
    prob: float = 0.0
    occ_dist: float = 0.0
    state: int = -1
    i: int = 0
    j: int = 0
    src_i: int = 0
    src_j: int = 0

class GridMapper(LaserScanner):
    def __init__(self, 
                 resolution: float,
                 xmin: float,
                 ymin: float,
                 xmax: float,
                 ymax: float,
                 props: LaserProperties,
                 Trs: Transform2D):
        super().__init__(props, Trs)

        self.prior_ = 0.5
        self.prob_occ_ = 0.90
        self.pro_free_ = 0.35
        self.log_odds_prior_ = prob_2_log_odds(self.prior_)
        self.log_odds_occ_ = prob_2_log_odds(self.prob_occ_)
        self.log_odds_free_ = prob_2_log_odds(self.pro_free_)

        self.resolution_ = resolution
        self.max_occ_dist_ = 10.0
        self.cell_radius_ = map_size(0.0, self.max_occ_dist_, resolution) 
        self.xmin_ = xmin
        self.ymin_ = ymin
        self.xmax_ = xmax
        self.ymax_ = ymax
        self.xsize_ = map_size(xmin, xmax, resolution)
        self.ysize_ = map_size(ymin, ymax, resolution)

        self.distance_ = [[0.0 for _ in range(self.cell_radius_)] for _ in range(self.cell_radius_)]
        total = self.xsize_ * self.ysize_
        self.map_: List[Cell] = [Cell(log_odds=self.log_odds_prior_, 
                                        prob=self.prior_,
                                        occ_dist=self.max_occ_dist_,
                                        state=-1) for _ in range(total)]
        
        self.occ_cells_ = set()
        self.free_cell_ = set()

        self.pre_compose_distance_field()
    
    def likelihood_field_model(self, beam_length: List[float], pose: Transform2D) -> float:
        if len(self.occ_cells_) == 0:
            return 1.0
        
        var_hits = self.sigma_hit * self.sigma_hit
        end_points = self.laser_end_points(beam_length, pose)

        p = 1.0

        for point in end_points:
            if isinstance(point, Vector2D):
                px, py = point.x, point.y
            else:
                px, py = float(point[0]), float(point[1])
            
            try:
                idx = self.world_2_row_major(px, py)
            except ValueError:
                continue

            z = self.map_[idx].occ_dist
            pz = 0.0
            pz += self.z_hit * pdf_normal(z, var_hits)
            pz += self.z_rand / self.range_max
            p *= pz
        
        return p
    
    def integrate_scan(self, beam_length: List[float], pose: Transform2D)->None:
        end_points = self.laser_end_points(beam_length, pose)
        for pt in end_points:
            if isinstance(pt, Vector2D):
                px, py = pt.x, pt.y
            else:
                px, py = float(pt[0]), float(pt[1])
            
            free_index: List[int] = []
            try:
                self.free_grid_index(free_index, Vector2D(px, py), pose)
            except Exception:
                free_index = []
            
            for idx in free_index:
                cell = self.map_[idx]
                cell.log_odds += (self.log_odds_free_ - self.log_odds_prior_)
                self.update_cell_state(cell, idx)
            
            try:
                occ_idx = self.world_2_row_major(px, py)
                occ_cell = self.map_[occ_idx]
                occ_cell.log_odds += (self.log_odds_occ_ - self.log_odds_prior_)
                self.update_cell_state(occ_cell, occ_idx)
            except ValueError:
                pass
        
        self.euclidean_signed_distance_field()
    
    def grid_map(self) -> List[int]:
        output = [-1] * len(self.map_)
        for i, cell in enumerate(self.map_):
            row = i // self.xsize_
            col = i % self.xsize_
            idx = col * self.xsize_ + row
            prob = cell.prob

            if abs(prob - self.prior_) < 1e-12:
                output[idx] = -1
            elif prob >= self.prob_occ_:
                output[idx] = 100
            elif prob <= self.pro_free_:
                output[idx] = 0
            else:
                output[idx] = int(prob * 100)
        
        return output
    
    def print_esdf(self) -> None:
        for i, cell in enumerate(self.map_):
            row = i // self.xsize_
            col = i % self.xsize_
            idx = row * self.xsize_ + col
            if col == self.xsize_ - 1:
                print()
    
    def pre_compose_distance_field(self)->None:
        for i in range(len(self.distance_)):
            for j in range(len(self.distance_)):
                self.distance_[i][j] = math.sqrt(i*i + j*j)
    
    def enqueue_cell(self, i: int, j: int, src_i: int, src_j: int, 
                     heap: List[Tuple[float, int]], marked: List[int])->None:
        idx = self.grid_2_row_major(i, j)
        if marked[idx]:
            return
        di = abs(i - src_i)
        dj = abs(j - src_j)

        try:
            dist = self.distance_[di][dj]
        except Exception:
            return
        
        if dist > self.cell_radius_:
            return
        
        self.map_[idx].occ_dist = dist * self.resolution_
        self.map_[idx].i = i
        self.map_[idx].j = j
        self.map_[idx].src_i = src_i
        self.map_[idx].src_j = src_j

        heapq.heappush(heap, (self.map_[idx].occ_dist, idx))
        marked[idx] = 1
    
    def euclidean_signed_distance_field(self)->None:
        if len(self.occ_cells_) == 0:
            return
        
        marked = [0] * (self.xsize_ * self.ysize_)
        heap: List[Tuple[float, int]] = []

        for key in self.occ_cells_:
            self.map_[key].occ_dist = 0.0
            self.map_[key].i = key // self.xsize_
            self.map_[key].j = key % self.xsize_
            self.map_[key].src_i = self.map_[key].i
            self.map_[key].src_j = self.map_[key].j
            marked[key] = 1
            heapq.heappush(heap, (0.0, key))
        while heap:
            current_dist, current_idx = heapq.heappop(heap)
            current_cell = self.map_[current_idx]
            i = current_cell.i
            j = current_cell.j

            if i > 0:
                self.enqueue_cell(i - 1, j, current_cell.src_i, current_cell.src_j, heap, marked)
            if j > 0:
                self.enqueue_cell(i, j - 1, current_cell.src_i, current_cell.src_j, heap, marked)
            if i < self.xsize_ - 1:
                self.enqueue_cell(i + 1, j, current_cell.src_i, current_cell.src_j, heap, marked)
            if j < self.ysize_ - 1:
                self.enqueue_cell(i, j + 1, current_cell.src_i, current_cell.src_j, heap, marked)

    def update_cell_state(self, cell: Cell, index: int)->None:
        prob = log_odds_2_prob(cell.log_odds)
        if abs(prob - self.prior_) < 1e-12:
            cell.state = -1
            cell.prob = self.prior_
            self.update_cell_hash(-1, index)
        elif prob > self.prob_occ_:
            cell.state = 1
            cell.prob = 1.0
            self.update_cell_hash(1, index)
        elif prob <= self.pro_free_:
            cell.state = 0
            cell.prob = 0.0
        else:
            cell.state = -1
            cell.prob = prob
            self.update_cell_hash(-1, index)
    
    def update_cell_hash(self, state: int, index: int) -> None:
        if state == 1:
            self.occ_cells_.add(index)
            self.free_cell_.discard(index)
        else:
            if index in self.occ_cells_:
                self.occ_cells_.remove(index)
            if state == 0:
                self.free_cell_.add(index)
            else:
                self.free_cell_.discard(index)

    def free_grid_index(self, free_index: List[int], point: Vector2D, pose: Transform2D)-> None:
        pose_x, pose_y = pose.v2.x, pose.v2.y
        pose_grid = self.world_2_grid(pose_x, pose_y)
        point_grid = self.world_2_grid(point.x, point.y)
        x0, y0 = pose_grid.i, pose_grid.j
        x1, y1 = point_grid.i, point_grid.j
        dx = x1 - x0
        dy = y1 - y0

        if dx == 0:
            step = 1 if dy > 0 else -1
            y_start = y0
            y_end = y1 + step  # To include up to but not including y1
            for y in range(y_start, y_end, step):
                free_index.append(self.grid_2_row_major(x0, y))
            if free_index: free_index.pop()  # Exclude endpoint
            return

        if dy == 0:
            step = 1 if dx > 0 else -1
            x_start = x0
            x_end = x1 + step
            for x in range(x_start, x_end, step):
                free_index.append(self.grid_2_row_major(x, y0))
            if free_index: free_index.pop()
            return
        
        if abs(dy) < abs(dx):
            if x0 > x1:
                free_index.append(self.grid_2_row_major(x0, y0))
                self.line_low(free_index, x1, y1, x0, y0)
            else:
                free_index.append(self.grid_2_row_major(x0, y0))
                self.line_low(free_index, x0, y0, x1, y1)
            return

        if abs(dx) < abs(dy):
            if y0 > y1:
                free_index.append(self.grid_2_row_major(x0, y0))
                self.line_high(free_index, x1, y1, x0, y0)
            else:
                free_index.append(self.grid_2_row_major(x0, y0))
                self.line_high(free_index, x0, y0, x1, y1)
            return

        if abs(dy) == abs(dx):
            self.line_diag(free_index, x0, y0, x1, y1)
            return
        
        raise ValueError("Bresenham algorithm failed")
    
    def line_low(self, free_index: List[int], x0: int, y0: int, x1: int, y1: int)-> None:
        dx = x1 - x0
        dy = y1 - y0
        yi = 1
        if dy < 0:
            yi = -1
            dy = -dy
        D = 2 * dy - dx
        y = y0
        ctr = 0
        for x in range(x0, x1):
            if ctr != 0:
                free_index.append(self.grid_2_row_major(x, y))
            if D > 0:
                y += yi
                D -= 2 * dx
            D += 2 * dy
            ctr += 1
                # D -= 2*dx
            D += 2*dy
            ctr += 1

    def line_high(self, free_index: List[int], x0: int, y0: int, x1: int, y1: int)-> None:
        dx = x1 - x0
        dy = y1 - y0
        xi = 1
        if dx < 0:
            xi = -1
            dx = -dx
        D = 2 * dx - dy
        x = x0
        ctr = 0
        for y in range(y0, y1):
            if ctr != 0:
                free_index.append(self.grid_2_row_major(x, y))
            if D > 0:
                x += xi
                D -= 2 * dy
            D += 2 * dx
            ctr += 1
    def line_diag(self, free_index: List[int], x0: int, y0: int, x1: int, y1: int)-> None:
    
        dx = x1 - x0
        dy = y1 - y0
        xi = -1 if dx < 0 else 1
        yi = -1 if dy < 0 else 1
        x, y = x0, y0
        while x != x1 and y != y1:
            free_index.append(self.grid_2_row_major(x, y))
            x += xi
            y += yi
        
    def world_2_grid(self, x: float, y: float)->GridCoordinates:
        if not(x > self.xmin_ and x < self.xmax_):
            raise ValueError("X position not in bounds")
        if not(y > self.ymin_ and y < self.ymax_):
            raise ValueError("Y position not in bounds")
        
        gi = int(math.floor(x - self.xmin_) / self.resolution_)
        if gi == self.xsize_:
            gi -= 1
        
        gj = int(math.floor(y - self.ymin_) / self.resolution_)
        if gj == self.ysize_:
            gj -= 1
        
        return GridCoordinates(gi, gj)
    
    def world_2_row_major(self, x: float, y: float)->int:
        if not(x > self.xmin_ and x < self.xmax_):
            raise ValueError("X position not in bounds")
        if not(y > self.ymin_ and y < self.ymax_):
            raise ValueError("Y position not in bounds")
        
        i = int(math.floor(x - self.xmin_) / self.resolution_)
        if i == self.xsize_:
            i -= 1
        
        j = int(math.floor(y - self.ymin_) / self.resolution_)
        if j == self.ysize_:
            j -= 1
        
        return self.grid_2_row_major(i, j)
    
    def grid_2_row_major(self, i: int, j: int)->int:
        return i * self.xsize_ + j