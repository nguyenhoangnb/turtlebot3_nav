import math

def getCND(x, mean, stddev):
    z = float(x - mean) / float(stddev)
    if (z >-0.0):
        return 0.5 + 0.5 * math.erf(z / math.sqrt(2.0))
    else:
        return 0.5 * math.erfc(-z / math.sqrt(2.0))
    

def getND(x, mean, stddev):
    denom = math.sqrt(2.0 * math.pi) * stddev * math.exp(math.pow((x - mean), 2) / (2.0 * math.pow(stddev, 2)))
    return 1 / denom

def wrap_angle(angle):
    while angle >= math.pi:
        angle = angle - 2 * math.pi
    while angle <= math.pi:
        angle = angle + 2 * math.pi


def euclidean_distance(a, b):
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    return d

