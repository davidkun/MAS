#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time, sys
from   scipy import linalg
import agent
from   simParams import *


def main():
  """
  This is a testing function to make sure
  the code is running properly.
  
  Run this to check for errors.
  """
  
  # Initial States
  IC1 = np.matrix([ [1],
                    [1],
                    [0],
                    [0] ])
  
  
  # Create Agent
  p1 = agent.point(IC1)

  # Controllability Grammian
  p1.Wc = gram(p1.A, p1.B, int(T/Ts))

  # Begin Simulation
  runSim(p1)
  
  
def runSim(leader):
  """
  Simulate vehicle movement in the xy-plane. 
  """
  
  fig = plt.figure()
  ax = fig.add_subplot(111, \
        autoscale_on=False, \
        xlim=(0, 10), ylim=(0, 10))
  plt.ion()
  plt.hold(True)
  abc = ax.plot(leader.X[0,0], leader.X[1,0], 'o')
  
  N = int(T/Ts)
  print N
  for k in range(N):
    plt.setp(abc[0], data=(leader.X[0,0],leader.X[1,0]))
    plt.draw()
    time.sleep(.2)
    
    # Controller Design
    u = leader.B.T * leader.A.T**(N-1-k) * \
        linalg.inv(leader.Wc) * np.r_[wPts[target].T,np.zeros([2,1])]
    
    leader.step(u)
  
  plt.ioff()
  
  # Plot Vehicle Path
  track = ax.plot(leader.Xhist[0:-1,0],leader.Xhist[0:-1,1])
  plt.setp(track, linestyle=':', color='b')
  
  # Show Plot
  plt.show()



def gram(A, B, N):
  """ 
  Wc = gram(A, B, N) computes the discrete time 
  Controllability Grammian of the system.
  """
  
  import numpy as np
  from scipy import linalg
  
  Wc = np.zeros(np.shape(A))
  
  for k in range(N):
    Wc = Wc + A**(N-1-k) * \
         B * B.T * \
         (A.T)**(N-1-k)
  
  return Wc

if __name__ == '__main__':
  main()

