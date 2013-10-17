#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time, sys
from   scipy import linalg
import agent
import cfg

def main():
  """
  This is a testing function to make sure
  the code is running properly.
  
  Run this to check for errors.
  """
  
  # Initial States
  IC1 = np.matrix([ [1.],
                    [1.],
                    [0.],
                    [0.] ])
  
  
  # Create Agent
  p1 = agent.point(IC1)

  # Controllability Grammian
  p1.Wc = gram(p1.A, p1.B, int(cfg.T/cfg.Ts))

  # Begin Simulation
  runSim(p1)


# # # # # # # # # # # # #
#                       #
# Supporting Functions: #
#                       #
# # # # # # # # # # # # #


def runSim(leader):
  """
  Simulate vehicle movement in the xy-plane. 
  """
  
  plotSim(leader, initial=True)
  
  N = int(cfg.T/cfg.Ts)
  print 'N =', N
  for k in range(N):

    plotSim(leader)
    
    # Controller Design
    u = leader.B.T * leader.A.T**(N-1-k+1) * \
        linalg.inv(leader.Wc) * \
        (np.r_[cfg.wPts[cfg.target].T,np.zeros([2,1])] - leader.X0)
    
    leader.step(u)
    
  plotSim(leader, final=True)
  

def plotSim(vehicle, initial=False, final=False):
  """
  Plot vehicle movement in the xy-plane.
  """

  if initial:
    # Create Figure
    cfg.fig = plt.figure()
    cfg.ax = cfg.fig.add_subplot(111, \
              autoscale_on=False, \
              xlim=(0, 10), ylim=(0, 10))
    plt.grid(True)
    plt.ion()
    
    # Add Waypoints to Figure
    plt.plot(cfg.wPts[:,0],cfg.wPts[:,1], 'o', color='gray', markersize=20)
      
    return
  
  #cfg.abc = cfg.ax.plot(vehicle.X[0,0], vehicle.X[1,0], 'bo')
  plt.plot(vehicle.X[0,0], vehicle.X[1,0], 'ob')
  #plt.setp(cfg.abc[0], data=(vehicle.X[0,0],vehicle.X[1,0]))
  plt.draw()
  time.sleep(.1)
  
  
  if final:
    plt.ioff()
    # Plot Vehicle Path
    #track = cfg.ax.plot(vehicle.Xhist[:,0],vehicle.Xhist[:,1])
    #plt.setp(track, linestyle=':', color='b')
    
    # Show Plot
    plt.show()


def gram(A, B, N):
  """ 
  Wc = gram(A, B, N) computes the discrete time 
  Controllability Grammian of the system.
  """
  
  Wc = np.zeros(np.shape(A))
  
  for k in range(N):
    Wc = Wc + A**(N-1-k+1) * \
         B * B.T * \
         (A.T)**(N-1-k+1)
  
  return Wc


if __name__ == '__main__':
  main()

