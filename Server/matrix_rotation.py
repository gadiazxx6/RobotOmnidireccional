#from dependencies.__init__ import *
import numpy as np
import math

def rot_x(alpha): #ROLL ROTANDO SOBRE X
    return np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)],
    ])

def rot_y(theta): #YAW ROTANDO SOBRE Y
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)],
    ])

def rot_z(beta): #PITCH ROTANDO SOBRE Z
    return np.array([
        [np.cos(beta), -np.sin(beta), 0],
        [np.sin(beta), np.cos(beta), 0],
        [0, 0, 1],
    ])
