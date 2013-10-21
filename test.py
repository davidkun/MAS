#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time, sys
from   scipy import linalg
import agent
import cfg

# add followers

def main():
  """
  This is a testing function to make sure
  the code works.
  """
  
  # Initial States
  IC1 = np.matrix([ [1.],
                    [1.],
                    [0.],
                    [0.] ])
  
  # Create Agent
  leader = agent.point( IC1 )
  
  # Begin Simulation
  runSim( leader )


# # # # # # # # # # # # #
#                       #
# Supporting Functions: #
#                       #
# # # # # # # # # # # # #



# # # # # # # # # # # # #
# Function:   runSim    #
# # # # # # # # # # # # #

def runSim(leader):
  """
  Simulate vehicle movement in the xy-plane. 
  """
  
  # Track Each Waypoint Sequentially
  for target in range(cfg.n_wPts):
    
    # Current Initial State
    X0 = leader.X
    
    # Xf velocities: u,v
    if (cfg.n_wPts - 1) > target:
      check = cfg.wPts[target+1,0] < cfg.wPts[target,0]
      u = 0.2 * np.cos( check*np.pi )
      check = cfg.wPts[target+1,1] < cfg.wPts[target,1]
      v = 0.2 * np.cos( check*np.pi )
    else:
      u, v = 0, 0
    
    # Final Desired State
    Xf = np.r_[cfg.wPts[target].T, \
               np.matrix([[u],[v]])]
    
    # Time Horizon for Current Waypoint
    N = cfg.N[target]
    
    # Controllability Grammian
    leader.Wc = gram( leader.A, leader.B, N )
    
    for k in range(N):
      # Control Input U[k]
      U = (leader.B.T) * (leader.A.T)**(N-1-k) * \
          linalg.inv(leader.Wc) * ( Xf - (leader.A**N)*X0 )
      # Implement Control
      leader.step( U )
  
  # Initialize Plot
  plotSim(leader, initial=True)
  # Animate Plot
  plotSim(leader, final=True)
  
  
# # # # # # # # # # # # #
# Function:   plotSim   #
# # # # # # # # # # # # #

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
    cfg.ax.grid(True)
    plt.ylabel('Y [m]',rotation=0)
    plt.xlabel('X [m]')
    
    # Add Waypoints to Figure
    plt.plot(cfg.wPts[:,0],cfg.wPts[:,1], 'o', color='0.75', markersize=23)
    #plt.ion()
    
    # Plot current location of vehicle
    cfg.line, = cfg.ax.plot(vehicle.X[0,0], vehicle.X[1,0], 'ob')
    #plt.draw()
    
    # Time
    cfg.timeStamp = 'Time = %.1f s'
    cfg.time_text = cfg.ax.text(0.05, 0.9, '', transform=cfg.ax.transAxes)
  
  if final:
    cfg.hist = vehicle.Xhist
    ani = animation.FuncAnimation( cfg.fig, 
                                   plotAnimate,
                                   np.arange(1,len(cfg.hist)),
                                   interval=cfg.Ts*1000,
                                   blit=True,
                                   init_func=plotInit )        
    # Plot Vehicle Path
    track = cfg.ax.plot(vehicle.Xhist[:,0],vehicle.Xhist[:,1])
    plt.setp(track, linestyle=':', color='b')
    
    # Show Animation
    plt.show()
    
    # Save Animation Video (requires ffmpeg, 
    # from https://github.com/FFmpeg/FFmpeg)
    #ani.save('waypointVid.mp4', writer="ffmpeg", fps=15)

# # # # # # # # # # # # #
# Function:   plotInit  #
# # # # # # # # # # # # #

def plotInit():
  """ 
  Initialize plot for animation.
  """
  cfg.line.set_data([], [])
  cfg.time_text.set_text('')
  return cfg.line, cfg.time_text


# # # # # # # # # # # # #
# Function: plotAnimate #
# # # # # # # # # # # # #

def plotAnimate(i):
  x0, y0 = cfg.hist[0,0], cfg.hist[0,1]
  x, y = cfg.hist[i,0], cfg.hist[i,1]
  cfg.line.set_data([x0,x], [y0,y])
  cfg.time_text.set_text(cfg.timeStamp%(i*cfg.Ts))
  return cfg.line, cfg.time_text

  
# # # # # # # # # # # # #
# Function:   gram      #
# # # # # # # # # # # # #

def gram(A, B, N):
  """ 
  Wc = gram(A, B, N) computes the discrete-time 
  Controllability Grammian of the system.
  """
  
  Wc = np.zeros_like(A)
  
  for k in range(N):
    Wc = Wc + A**(N-1-k) * B*B.T * (A.T)**(N-1-k)
  
  return Wc


# # # # # # # # # # # # #
# # # # # # # # # # # # #

if __name__ == '__main__':
  main()

# # # # # # # # # # # # #
# # # # # # # # # # # # #