#!/usr/bin/env python

import numpy as np

"""
This contains simulation parameters that need to be
globally available.
"""

# Waypoints = [x, y]
wPts = np.matrix([ [6., 6.],
                   [1., 8.],
                   [9., 9.] ])
n_wPts = np.shape(wPts)[0]

# Simulation Time [s]
T = 15.0

# Sample Time [s]
Ts =  0.1

# Time Horizon for Each Waypoint [s]
N_time = [5., 7., 3.]

# Time Horizon Index [k]
N = [int(N_time[0]/Ts), 
     int(N_time[1]/Ts),
     int(N_time[2]/Ts)]