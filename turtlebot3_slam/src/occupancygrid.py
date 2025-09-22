import math
import numpy
from fastslam_utilties import *

class OccupancyGrid(object):
    def __init__(self, dimensions, step):

        self.dimensions = dimensions
        self.origin = (int(dimensions[0] / step), int(dimensions[1] / step))
        if step > 1:
            raise TypeError("Step size cannot be greater than one")
        self.step = step
        self.map = [GridList(self.origin, self.dimensions, self.step) for i in range(0, int(2*self.dimensions[0] / self.step) + 1)]
    
    def __getitem__(self, key):
        if abs(int(key)) <= self.dimensions[0]:
            index = int(key / self.step) + self.origin[0]
            return self.map[index]
        else:
            raise IndexError
        



class GridList(list):
    def __init__(self, origin, dimensions, step):
        self.data = [None for i in range(0, int(2*dimensions[1]/step) + 1)]
        self.dimensions = dimensions
        self.origin = origin
        self.step = step
    
    def __getitem__(self, key):
        if abs(int(key)) <= self.dimensions[1]:
            index = int(key / self.step) + self.origin[1]
            return self.data[index]
        else:
            raise IndexError
        
    def __setitem__(self, key, item):
        if abs(int(key)) <= self.dimensions[1]:
            index = int(key/self.step) + self.origin[1]
            self.data[index] = item
        else:
            raise IndexError
    
    def __contains__(self, key):
        return abs(int(key[1])) <= self.dimensions
    
    def __iter__(self):
        for item in self.data:
            yield item