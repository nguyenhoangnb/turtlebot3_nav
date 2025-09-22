import math
import numpy as np
PI = math.pi


def most_equal(d1, d2, epsilon=1.0e-12):
    #Compare two floating-point number
    return abs(d1 - d2) < epsilon


def deg2rad(deg):
    # Convert degree to radian
    return deg * PI / 180

def rad2deg(rad):
    #convert radian to degree
    return rad * 180 / PI

class Vector2D:
    
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __str__(self):
        return f"[{self.x} {self.y}]"
    
    @staticmethod
    def unit_vector(v):
        mag = math.sqrt(v.x**2 + v.y**2)
        if mag == 0:
            return Vector2D(0.0, 0.0)
        
        return Vector2D(v.x / mag, v.y / mag)
    
class Twist2D:
    def __init__(self, theta_dot = 0.0, x_dot = 0.0, y_dot = 0.0):
        self.theta_dot = theta_dot
        self.x_dot = x_dot
        self.y_dot = y_dot

    def __str__(self):
        return f"[{self.theta_dot} {self.x_dot} {self.y_dot}]"
    
class Transform2D:
    def __init__(self, trans=None, radians=0.0):
        if trans is None:
            trans = Vector2D(0.0, 0.0)
        self.v2 = Vector2D(trans.x, trans.y)
        self.theta = radians

    def __call__(self, v : Vector2D):
        #Apply transformation to a Vector2D
        if isinstance(v, Vector2D):
            x_new = math.cos(self.theta) * v.x - math.sin(self.theta) * v.y + self.v2.x
            y_new = math.sin(self.theta) * v.x + math.cos(self.theta) * v.y + self.v2.y  # Fixed: was v2.x
            return Vector2D(x_new, y_new)
        elif isinstance(v, Twist2D):
            theta_dot = v.theta_dot
            x_dot = self.v2.x  * v.theta_dot + v.x_dot * math.cos(self.theta) - v.y_dot * math.sin(self.theta)
            y_dot = -self.v2.x  * v.theta_dot + v.x_dot * math.sin(self.theta) + v.y_dot * math.cos(self.theta)
            return Twist2D(theta_dot, x_dot, y_dot)
    
    def inv(self):
        # invert transformation
        inv_x = -(self.v2.x * math.cos(self.theta)) - (self.v2.y * math.sin(self.theta))
        inv_y = -(self.v2.y * math.cos(self.theta)) + self.v2.x * math.sin(self.theta)
        return Transform2D(Vector2D(inv_x,  inv_y), -self.theta)
    
    def __imul__(self, rhs):
        new_x = self.v2.x + rhs.v2.x * math.cos(self.theta) - rhs.v2.y * math.sin(self.theta)
        new_y = self.v2.y + rhs.v2.x * math.sin(self.theta) + rhs.v2.y * math.cos(self.theta)
        self.v2.x = new_x
        self.v2.y = new_y
        self.theta += rhs.theta
        return self
    def __mul__(self, rhs):
        result = Transform2D(Vector2D(self.v2.x, self.v2.y), self.theta)
        result *= rhs
        return result
    
    def compose(self, other):
        """Compose this transform with another: result = self * other"""
        return self * other
    
    def translation(self)->Vector2D:
        return Vector2D(self.v2.x, self.v2.y)
    
    def rotation(self)->float:
        return self.theta
    
    def new_twist(self, v:Twist2D)->Twist2D:
        theta_dot = v.theta_dot
        x_dot = self.v2.x  * v.theta_dot + v.x_dot * math.cos(self.theta) - v.y_dot * math.sin(self.theta)
        y_dot = -self.v2.x  * v.theta_dot + v.x_dot * math.sin(self.theta) + v.y_dot * math.cos(self.theta)
        
        return Twist2D(theta_dot, x_dot, y_dot)
    
    def to_matrix(self) -> np.ndarray:
        ctheta = math.cos(self.theta)
        stheta = math.sin(self.theta)
        T_matrix = np.array([
            [ctheta, -stheta, 0, self.v2.x],
            [stheta,  ctheta, 0, self.v2.y],  # Fixed: was -stheta
            [0,        0,      1, 0],
            [0,        0,      0, 1]
        ])
        return T_matrix

    def __str__(self):
        return f"deg: {rad2deg(self.theta)} x: {self.v2.x} y: {self.v2.y}"
    
def intergrate_twist(V: Twist2D):
    if V.theta_dot == 0.0:
        return Transform2D(Vector2D(V.x_dot, V.y_dot))
    else:
        v_x = V.x_dot / V.theta_dot
        v_y = V.y_dot / V.theta_dot
        theta = V.theta_dot

        Tsb = Transform2D(Vector2D(v_x, v_y), 0.0)
        Tss_ =  Transform2D(Vector2D(0.0, 0.0), theta)
        Tbs = Tsb.inv()
        
        return Tbs * Tss_ * Tsb
    

def normalize_angle(theta):
    while theta > math.pi:
        theta -= 2 * math.pi
    
    while theta < -math.pi:
        theta += 2*math.pi
    
    return theta
        
