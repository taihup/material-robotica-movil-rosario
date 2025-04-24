""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
    Modified by Zoey Chen for CSE590A: Probabilistic Robotics (Spring 2023)
"""

import numpy as np
import matplotlib.pyplot as plt
import math

def minimized_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    while angle < -np.pi:
        angle += 2 * np.pi
    while angle >= np.pi:
        angle -= 2 * np.pi
    return angle
