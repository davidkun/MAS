#!/usr/bin/env python

import numpy as np

"""
This contains simulation parameters that need to be
globally available.
"""

# Initial States
IC1 = np.matrix([ [1.],
                  [1.],
                  [0.],
                  [0.] ])

IC2 = np.matrix([ [-1.],
                  [4.],
                  [0.],
                  [0.] ])

IC3 = np.matrix([ [-1.],
                  [.5],
                  [0.],
                  [0.] ])

IC4 = np.matrix([ [3.],
                  [.5],
                  [0.],
                  [0.] ])

# Waypoints = [x, y]
wPts = np.matrix([ [6., 6.],
                   [1., 7.],
                   [5., 13.] ])

n_wPts = np.shape(wPts)[0]

# Simulation Time [s]
T = 18.

# Sample Time [s]
Ts =  0.05

# Time Horizon for Each Waypoint [s]
N_time = [5., 7., 6., 4.5]

# Time Horizon Index [k]
N = [int(N_time[0]/Ts), 
     int(N_time[1]/Ts),
     int(N_time[2]/Ts),
     int(N_time[3]/Ts)]

# Reference Distances for Followers (relative to leader)
# (currently set up for 3 followers)
bodyRef = np.matrix(np.zeros((4,3)))
r = np.matrix([ [-1.0 , -1.0 , -1.8], 
                [-0.5 ,  0.5 ,  0.0],  ])

# Q,R Cost Matrices for Followers' Control Law
Q = np.matrix([ [2.,0,0,0],
                [0,2.,0,0],
                [0,0,1.,0],
                [0,0,0,1.] ])

R = np.matrix([ [.1 , 0  ],
                [0  , .1 ] ])

# Some Extra Conditions
#
# bool if leader waits at final waypoint
stay = False        
# wait for 8 seconds
stayTime = int(8/Ts)
# type of KF that leader uses ('--simple' / '--adv')
leaderEstType = '--simple'
# bool for intial advanced leader estimation
leaderEstSwitch = 1