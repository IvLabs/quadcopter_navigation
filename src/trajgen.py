import numpy as np
import matplotlib.pyplot as plt
from bspline import approximate_b_spline_path,interpolate_b_spline_path
from genmap import generatemap


def trajgenlemniscate():
    t = 0
    waypoints_t = np.linspace(t, t + 2*np.pi, 40)
    x = 5*np.cos(waypoints_t)
    y = 7*np.sin(2*waypoints_t)
    x = x.tolist()
    y = y.tolist()
    xtraj,ytraj = approximate_b_spline_path(x,y,100)
    return xtraj,ytraj

def trajcircle():
    t = 0.0
    waypoints_t = np.linspace(t, t + 2*np.pi, 40)
    x = 5*np.cos(waypoints_t)
    y = 5*np.sin(waypoints_t)
    x = x.tolist()
    y = y.tolist()
    xtraj,ytraj = approximate_b_spline_path(x,y,50)
    return xtraj,ytraj