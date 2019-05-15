"""
Path tracking simulation with pure pursuit steering control and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
Edited: Arsalan Akhter
"""
import numpy as np
import math
import matplotlib.pyplot as plt

k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 0.5  # speed propotional gain
dt = 0.4  # [s]
L = 0.675/2  # [m] wheel base of vehicle


show_animation = False
class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0,):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w

def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    #if delta > 3.14:
    #    delta -= 6.28
    #if delta < -3.14:
    #    delta += 6.28


    return delta, ind


def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind
