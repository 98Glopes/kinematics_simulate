import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os

class State:

    def __init__(self, max_angle=math.pi/12, dt=1e-3, x=0.0, y=0.0, yaw=0.0, v=0.0):
        
        #Physical characterists
        self.length = 1.5 # [m] vehicle length
        self.lr = self.length / 2
        self.lf = self.length / 2
        self.max_angle = max_angle # [rad] max steering angle

        #initial position states
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.dt = dt
        self.first_time = 0

    def update(self, v, delta, simulation_time):
        beta = math.atan((self.lr/self.length) * math.tan(delta))

        self.x += self.v * math.cos(self.yaw + beta) * self.dt
        self.y += self.v * math.sin(self.yaw + beta) * self.dt
        self.yaw += ((math.cos(beta) * math.tan(delta)) / self.length)  * \
            self.v * self.dt
        self.v = v
        self.delta = delta
        self.simulation_time = simulation_time

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
    
    @property
    def query_state(self):
        """
        return a string in this format:
        X;Y;YAM;V;DELTA
        """
        return '%.3f;%.3f;%.3f;%.3f;%.3f;%.3f' % (self.x, self.y, self.yaw, self.v, self.delta, self.simulation_time)
    
    def export_data(self, fps):

        if self.simulation_time > (self.first_time + 1/fps):
            self.first_time = self.simulation_time
            return self.query_state
        else:
            return None



class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def ramp(t, dx, initial=0, max=None):
    """
    Generate a ramp signal
    if max=None the return value tends to infinite
    :param t: float: simulation time
    :param dx: float: angular coeficient 
    :param initial: float: initial value of the fn
    :param max: float: max value of the fn
    """

    value = t*dx + initial
    if max == None:
        return value
    elif value > max:
        return max
    else:
        return value

def sin(t, w, phase=0):
    return math.sin(w*t + phase)

def step(t, ofset):

    if t >= ofset:  
        return 1
    else:
        return 0