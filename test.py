#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time, sys
from   scipy import linalg
import agent
import cfg


def main():
  """
  This is a testing function to make sure
  the program works.
  """
  
  # Create Leader Agent
  leader = agent.point( cfg.IC1 )
  
  # Create Follower(s)
  f1 = agent.point( cfg.IC2 )
  f2 = agent.point( cfg.IC3 )
  f3 = agent.point( cfg.IC4 )
  
  # Begin Simulation
  runSim( leader, f1, f2, f3 )


# # # # # # # # # # # # # # # # # # # # # # # # # #
#                                                 #
#        Supporting Functions:                    #
#                                                 #
# # # # # # # # # # # # # # # # # # # # # # # # # # 



# # # # # # # # # # # # #
# Function:   runSim    #
# # # # # # # # # # # # #

def runSim(leader, *follower):
  """
  Simulate vehicles' movement in the xy-plane. 
  """
  
  # Number of Followers
  cfg.numFollowers = len(follower)
  
  # Track Each Waypoint Sequentially
  for target in range(cfg.n_wPts):
    
    # Current Initial State
    X0 = leader.X
    
    # Determine Xf velocities: u,v
    if (cfg.n_wPts - 1) > target:
      check = cfg.wPts[target+1,0] < cfg.wPts[target,0]
      u = 0.4 * np.cos( check*np.pi )
      check = cfg.wPts[target+1,1] < cfg.wPts[target,1]
      v = 0.4 * np.cos( check*np.pi )
    else:
      u, v = 0, 0
    
    # Final Desired State
    Xf = np.r_[cfg.wPts[target].T, \
               np.matrix([[u],[v]])]
    
    # Time Horizon for Current Waypoint
    N = cfg.N[target]
    
    # Controllability Grammian (Leader)
    leader.Wc = gram( leader.A, leader.B, N )
    
    #                                                    #
    # Make the next 2 for-loops as their own functions   #
    #                                                    #
    
    for k in range(N):
      # Implement Control U[k] (Leader)
      U = (leader.B.T) * (leader.A.T)**(N-1-k) * \
          linalg.inv(leader.Wc) * ( Xf - (leader.A**N)*X0 )
      leader.step( U )
      # Implement Control (Followers)
      if cfg.numFollowers > 0:
        for i in range(cfg.numFollowers):
          if i == 0:
            K = dare(leader, follower[i], cfg.Q, cfg.R, i)
          Uf = -K * (follower[i].X - (leader.X - cfg.r[:,i]))
          follower[i].step( Uf )
     
  if cfg.stay == True:
    for k in range(cfg.stayTime):
      leader.step( 0 )
      if cfg.numFollowers > 0:
        for i in range(cfg.numFollowers):
          Uf = -K * (follower[i].X - (leader.X - cfg.r[:,i]))
          follower[i].step( Uf )
  
  # Initialize Plot
  plotSim(leader, follower, initial=True)
  
  # Animate Plot
  plotSim(leader, follower, final=True)
  
  
# # # # # # # # # # # # #
# Function:   plotSim   #
# # # # # # # # # # # # #

def plotSim(leader, follower, initial=False, final=False):
  """
  Plot vehicle movement in the xy-plane.
  """
  
  if initial:
    # Create Figure
    cfg.fig = plt.figure()
  
    cfg.ax = cfg.fig.add_subplot(111, \
          autoscale_on=False, \
          xlim=(-2, 12), ylim=(0, 14))
    cfg.ax.grid(True)
    plt.ylabel('Y [m]',rotation=0)
    plt.xlabel('X [m]')
    
    # Add Waypoints to Figure
    plt.plot(cfg.wPts[:,0],cfg.wPts[:,1], 'o', color='0.75', markersize=23)
    
    # Add Initial Vehicle Location(s)
    plt.plot(leader.X0[0,0], leader.X0[1,0], 'o', color='lightskyblue')
    if len(follower)>0:
      for i in range(len(follower)):
        plt.plot(follower[i].X0[0,0], follower[i].X0[1,0], 'D', \
                 color='lightgreen')
    
    # Initialize Vehicle Location on Figure
    cfg.point, = cfg.ax.plot([], [], 'ob')
    cfg.pointf, = cfg.ax.plot([], [], 'D', color='mediumseagreen')
    cfg.track, = cfg.ax.plot([], [], '-k')
    
    # Initialize Timestamp on Figure
    cfg.timeStamp = 'Time = %.1f s'
    cfg.time_text = cfg.ax.text(0.05, 0.9, '', transform=cfg.ax.transAxes)
  
  if final:
    cfg.hist = leader.Xhist
    if len(follower)>0:
      for i in range(len(follower)):
        cfg.hist = np.c_[cfg.hist, follower[i].Xhist]
    
    ani = animation.FuncAnimation( cfg.fig, 
                                   plotAnimate,
                                   np.arange(1,len(cfg.hist)),
                                   interval=cfg.Ts*1000,
                                   blit=True,
                                   init_func=plotInit )        
    # Plot Vehicle Path
    track = cfg.ax.plot(leader.Xhist[:,0],leader.Xhist[:,1])
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
  cfg.point.set_data([], [])
  cfg.pointf.set_data([], [])
  cfg.time_text.set_text('')
  return cfg.point, cfg.pointf, cfg.time_text


# # # # # # # # # # # # #
# Function: plotAnimate #
# # # # # # # # # # # # #

def plotAnimate(i):
  #                                                    #
  # ** add communication range circle around leader ** #
  #                                                    #
  R = range(cfg.numFollowers+1)
  cfg.point.set_data(cfg.hist[i,0], cfg.hist[i,1])
  R.pop(0)
  R = np.matrix(R)
  cfg.pointf.set_data(cfg.hist[i,R*2], cfg.hist[i,R*2+1])
  cfg.time_text.set_text(cfg.timeStamp%(i*cfg.Ts))
  return cfg.point, cfg.pointf, cfg.time_text

  
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
# Function:   dare      #
# # # # # # # # # # # # #

def dare(leader, f, Q, R, i):
  """ 
  Uf = dare(leader, f, Q, R, i) computes the Discrete time
  Algebraic Riccatti Equation (DARE):
    X = A'XA - (A'XB) * (R+B'XB)^-1 * (B'XA) + Q
  
  Then computes and returns the gain, K:
    K = (B'XB + R)^-1 * (B'XA)
  
  The gain can be  used to find:
    Uf = -K * (f.X - (leader.X - cfg.r))
  
  which is the control input required for the followers to 
  track a leader to within a pre-specified distance, cfg.r.
  """
  
  X = linalg.solve_discrete_are(f.A, f.B, Q, R)
  K = linalg.inv(f.B.T * X * f.B + R) * (f.B.T * X * f.A)
  return K


# # # # # # # # # # # # #
# # # # # # # # # # # # #

if __name__ == '__main__':
  main()

# # # # # # # # # # # # #
# # # # # # # # # # # # #