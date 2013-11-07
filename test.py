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
  
  # Run Simulation
  runSim( leader, f1, f2, f3 )
  
  # Plot Error
  #np.savetxt("e.csv", leader.e, delimiter=",")
  fig = plt.figure()
  plt.plot(np.matrix(np.arange(0.0,cfg.T,cfg.Ts)), leader.e[0,:], '.b')
  plt.show()
  
  
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
  
  # Error Storage
  leader.e = np.matrix(np.zeros((4,int(cfg.T/cfg.Ts))))
  e_count  = 0
  
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
    
    for k in range(N):
      if k == 0:
        # Get the Feedback Gain, K
        K = fbGain( leader, cfg.Q, cfg.R )
        # Get the simple Kalman Gain, Ls, using regular Filter
        Ls = kalGain( leader )
        # Get the advanced Kalman Gain, La, using improved Filter
        # La = ?????
        
        # Initialize Estimates
        leader.xh_old = np.matrix(np.zeros((4,1)))
        leader.u_old  = np.matrix(np.zeros((2,1)))
        
      # Leader's Observation of Itself: y11
      leader.y11 = leader.selfObserve()
      # Leader's Observation of Follower: y12
      leader.y12 = follower[0].observe()
      
      # 
      #Aa = np.r_[ np.c_[ leader.A, np.zeros_like(leader.A) ],
      #            np.c_[ -leader.B*K*leader.C, leader.A+leader.B*K*leader.C ] ]
      
      # New Estimate
      leader.xh_new = leaderEst( leader, Ls, cfg.leaderEstType)
      
      # Compute and Store Error
      e = leader.xh_new - leader.X
      leader.e[:,e_count] = e[:,0]
      e_count = e_count + 1
      
      # Implement Leader's Control
      U = (leader.B.T) * (leader.A.T)**(N-1-k) * \
          linalg.inv(leader.Wc) * ( Xf - (leader.A**N)*X0 )
      leader.step( U )
      
      # Update Estimates
      leader.xh_old = leader.xh_new
      #leader.u_old  = leader.u_new
      
      # Implement Followers' Control
      if cfg.numFollowers > 0:
        for i in range(cfg.numFollowers):
          if k == 0:
            # Get the Follower Feedback Gain, Kf
            Kf = fbGain( follower[i], cfg.Q, cfg.R )
            # Get the Follower Kalman Gain, Lf
            Lf = kalGain( follower[i] )
            # Initialize Estimates
            follower[i].xh_old = np.matrix(np.zeros((4,1)))
            follower[i].u_old  = np.matrix(np.zeros((2,1)))
          
          # Follower's Observation of Leader: y21
          follower[i].y21 = leader.observe()
          # Follower's Observation of Itself: y22
          follower[i].y22 = follower[i].selfObserve()
          
          # New Estimate
          follower[i].xh_new = followerEst( follower[i], Lf )
          
          # Convert Reference Distance to Body-Frame
          #cfg.bodyRef[:,i] = leaderFrame( follower[i].y21, i )
          cfg.bodyRef[0:2,i] = cfg.r[:,i]
          
          # Obtain New Control Input
          follower[i].u_new = -Kf * (follower[i].C * follower[i].xh_new - 
                                    (follower[i].y21 + cfg.bodyRef[:,i]))
          # Implement New Control
          follower[i].step( follower[i].u_new )
          # Update Estimates
          follower[i].xh_old = follower[i].xh_new
          follower[i].u_old  = follower[i].u_new
  
  #if cfg.stay == True:
  #  for k in range(cfg.stayTime):
  #    leader.step( 0 )
  #    if cfg.numFollowers > 0:
  #      for i in range(cfg.numFollowers):
  #        Uf = -K * (follower[i].X - (leader.X + cfg.bodyRef[:,i]))
  #        follower[i].step( Uf )
  
  
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
          xlim=(-2, 14), ylim=(-2, 14))
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
                                   interval=50,
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
# Function:   fbGain    #
# # # # # # # # # # # # #

def fbGain(f, Q, R):
  """ 
  K = fbGain(f, Q, R) solves the Discrete time
  Algebraic Riccatti Equation (DARE) for X:
    X = A'XA - (A'XB) * (R+B'XB)^-1 * (B'XA) + Q
  
  Then computes and returns the feedback gain, K:
    K = (B'XB + R)^-1 * (B'XA)
  
  The feedback gain can be  used to find the control input:
    u = -K * (f.X - (leader.X - r))
  
  which is the control input required for the followers to 
  track a leader to within a pre-specified distance, r,
  
  f : agent
  Q : cost matrix
  R : cost matrix
  """
  
  X = linalg.solve_discrete_are( f.A, f.B, Q, R )
  K = linalg.inv( f.B.T * X * f.B + R ) * ( f.B.T * X * f.A )
  return K


# # # # # # # # # # # # #
# Function:   kalGain   #
# # # # # # # # # # # # #

def kalGain(f):
  """
  L = kalGain(f) solves the Discrete-time Algebraic Riccatti 
  Equation (DARE) for X:
    X = AXA' - (AXC') * (R+CXC')^-1 * (CXA') + Q
  
  Then computes and returns the steady-state Kalman gain, L:
    L = XC' * (CXC'+R)^-1
  
  f : agent
  Q : covariance of state error
  R : covariance of measurement error
  """
  
  Q = np.matrix( f.stateCov  )
  R = np.matrix( f.sensorCov )
  X = linalg.solve_discrete_are( f.A.T, f.C.T, Q, R )
  L = X * f.C.T * linalg.inv( f.C * X * f.C.T + R )
  return L


# # # # # # # # # # # # # # #
# Function:  followerEst    #
# # # # # # # # # # # # # # #


def followerEst(f, L):
  """
  xh_new = followerEst(f, L) used inside a for-loop will
  recursively estimate the state of the follower using a
  typical Kalman Filter algorithm:
  
  xh_new = A*xh_old + B*u_old + L*(y22 - C*A*xh_old - C*B*u_old)
  
  f      :  follower (agent)
  L      :  Kalman gain
  xh_new :  new estimate of x
  xh_old :  old estimate of x
  y22    :  measured output of itself
  u_old  :  previous control input
  """
  
  xh_new = f.A * f.xh_old + f.B * f.u_old + \
           L * ( f.y22 - f.C * f.A * f.xh_old - f.C * f.B * f.u_old )
  return xh_new

# # # # # # # # # # # # # # #
# Function:  leaderEst      #
# # # # # # # # # # # # # # #


def leaderEst(l, Ls, kalType):
  """
  xh_new = leaderEst(l, L, kalType) is the leader's estimation method.
  
  If kalType == 'simple', the Leader will estimate its state
  only based on its own observation. The Kalman gain must be
  provided as well.
  
  If kalType == 'adv', the Leader wil estimate its state using
  its own observations AND the followers' observation.
  """
  
  if kalType == 'simple':
    xh_new = l.A * l.xh_old + l.B * l.u_old + \
             Ls * ( l.y11 - l.C * l.A * l.xh_old - l.C * l.B * l.u_old )
  if kalType == 'adv':
    xh_new = 0
  
  return xh_new
  
# # # # # # # # # # # # # # #
# Function:  leadFrame      #
# # # # # # # # # # # # # # #

def leaderFrame(leader, i):
  """
  bodyRef = leaderFrame(leader, i)
  
  Converts the global reference distance to body-frame of the leader
  for each i-th follower, based on observed direction of Leader's movement.
  """
  
  u = leader[2,0]
  v = leader[3,0]
  rot = np.matrix([ [u, -v],
                    [v,  u] ])
  rotNorm = (1 / linalg.det(rot)**0.5) * rot
  bodyRef = rotNorm * cfg.r[:,i]
  return np.r_[bodyRef, np.zeros((2,1))]


# # # # # # # # # # # # #
# # # # # # # # # # # # #

if __name__ == '__main__':
  main()

# # # # # # # # # # # # #
# # # # # # # # # # # # #