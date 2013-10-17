#!/usr/bin/env python

import numpy as np

"""
This contains simulation parameters that need to be
globally available.
"""

# Simulation Time [s]
T = 10.0

# Sample Time     [s]
Ts =  0.5

# Waypoints
target = 0
wPts = np.matrix([ [6., 6.],
                   [1., 4.] ])
