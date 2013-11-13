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
  
  # Default Algorithm is Simple KF
  if '--adv' in sys.argv:
    cfg.leaderEstType = '--adv'
    advOn = True
  else:
    cfg.leaderEstType = '--simple'
    advOn = False
  # Default is not to Save Error
  if '--save' in sys.argv:
    saveOn = True
  else:
    saveOn = False
  # Default is not to Plot Animation & Error
  if '--plot' in sys.argv:
    plotOn = True
  else:
    plotOn = False
  
  # Create Leader Agent
  leader = agent.point( cfg.IC1, sensorCov=np.identity(4)*0.1, stateCov=np.identity(4)*0.1 )
  
  # Create Follower(s)
  f1 = agent.point( cfg.IC2 )
  f2 = agent.point( cfg.IC3 )
  f3 = agent.point( cfg.IC4 )
  
  # Run Simulation
  runSim( leader, f1, f2, f3 )
    
  # Save Error
  if saveOn and not advOn:
    fhandle = file('data/e_simple.csv','a')
    np.savetxt(fhandle, cfg.e, delimiter=",")
  elif saveOn and advOn:
    fhandle = file('data/e_adv.csv','a')
    np.savetxt(fhandle, cfg.e, delimiter=",")
  
  # Simulation Animation, Error Plot
  if plotOn:
    plotSim( leader, f1, f2, f3 )
    plotError()
  
  
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
  
  # Total Time Steps
  N_total  = int(cfg.T/cfg.Ts)
    
  # Number of Followers
  cfg.numFollowers = len(follower)
  
  # Error Storage
  cfg.e = np.matrix(np.zeros((1,N_total)))
  e_cnt  = 0
  
  # Track Each Waypoint Sequentially
  for target in range(cfg.n_wPts):
    
    # Current Initial State
    if target==0:
      X0 = leader.X0
    else:
      X0 = leader.xh_new
    
    # Determine Xf velocities: u,v
    if (cfg.n_wPts - 1) > target:
      check = cfg.wPts[target+1,0] < cfg.wPts[target,0]
      u = 0.4 * np.cos( check*np.pi )
      check = cfg.wPts[target+1,1] < cfg.wPts[target,1]
      v = 0.4 * np.cos( check*np.pi )
      #u,v = 0.0, 0.0
    else:
      u, v = 0, 0
    
    # Final Desired State
    Xf = np.r_[cfg.wPts[target].T, np.matrix([[u],[v]])]
    
    # Time Horizon for Current Waypoint
    N = cfg.N[target]
    
    # Leader's Controllability Grammian
    leader.Wc = gram( leader.A, leader.B, N )
    # Leader's Feedback Gain, K
    K = fbGain( leader, cfg.Q, cfg.R )
    # Leader's simple Kalman Gain, Ls, using regular Filter
    leader.Ls = kalGain( leader )    
    
    for k in range(N):        
      # Leader's Observation of Itself: y11
      leader.y11 = leader.selfObserve()
      # Leader's Observation of Follower: y12
      leader.y12 = follower[0].observe()
      
      # New Estimate
      if cfg.leaderEstType == '--simple':
        leader.xh_new = leaderEst( leader )
      elif cfg.leaderEstType == '--adv':
        leader.xh_new = leaderEst( leader, follower )
      else:
        print '\n\ninvalid argument: '+cfg.leaderEstType+'\n\n'
        sys.exit(0)
      
      # Compute and Store Error
      e              = leader.xh_new - leader.X
      cfg.e[0,e_cnt] = e.T*e
      e_cnt          = e_cnt + 1
      
      # Leader's Reference, x_ref
      if k==0:
        leader.Xref_new = X0
      else:
        leader.Xref_new = leader.A*leader.Xref_old + leader.B*u_ref
      
      # Leader's Tracking Control, u_ref  
      u_ref = (leader.B.T) * (leader.A.T)**(N-1-k) * linalg.inv(leader.Wc) * \
              ( Xf - (leader.A**N)*X0 )
      # Leader's Total Control Input
      leader.u_new =  -u_ref - K*( leader.C*leader.xh_new - leader.C*leader.Xref_new )
      # Implement Control Input
      leader.step( leader.u_new )
      
      # Update Estimates
      leader.xh_old = leader.xh_new
      leader.u_old  = leader.u_new
      leader.Xref_old = leader.Xref_new
      
      # Implement Followers' Control
      if cfg.numFollowers > 0:
        for i in range(cfg.numFollowers):
          if k == 0:
            # Get the Follower Feedback Gain, Kf
            follower[i].K = fbGain( follower[i], cfg.Q, cfg.R )
            Kf            = follower[i].K
            # Get the Follower Kalman Gain, Lf
            follower[i].L = kalGain( follower[i] )
            Lf            = follower[i].L
          
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
  
  
# # # # # # # # # # # # #
# Function:   plotSim   #
# # # # # # # # # # # # #

def plotSim(leader, *follower):
  """
  Plot vehicle movement in the xy-plane.
  """
  
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
  #ani.save('sim1.mp4', writer="ffmpeg", fps=15)


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

def fbGain(f, Q=cfg.Q, R=cfg.R):
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


def leaderEst(*agents):
  """
  xh_new = leaderEst(*agents) is the leader's estimation method.
  
  If only the leader is passed, the leader will estimate its state
  only based on its own observation.
  
  If leader and followers are passed, the leader will estimate its 
  state using its own observations AND the followers' observation.
  """
  
  # Simple Estimation
  if len(agents)==1:
    l = agents[0]
    xh_new = l.A * l.xh_old + l.B * l.u_old + \
             l.Ls * ( l.y11 - l.C * l.A * l.xh_old - l.C * l.B * l.u_old )
  
  # Advanced Estimation
  if len(agents)>1:
    l  = agents[0]
    f  = agents[1][0]
    
    if cfg.leaderEstSwitch:
      # Obtain Leader's and Follower's Data
      K2 = fbGain(f)
      L2 = kalGain(f)
      Q1 = np.matrix( l.stateCov  )
      Q2 = np.matrix( f.stateCov  )
      R2 = np.matrix( f.sensorCov )
      X  = linalg.solve_discrete_are( f.A.T, f.C.T, Q2, R2 )
      P  = (np.matrix(np.identity(4)) - L2*f.C) * X
      
      # Build Augment Matrices
      cfg.Abar = np.r_[ np.c_[ l.A        , np.zeros_like(l.A) ],
                        np.c_[ f.B*K2*f.C, f.A-f.B*K2*f.C     ] ]
      cfg.Bbar = np.r_[ np.c_[ l.B               , np.zeros((4,4)) ],
                        np.c_[ np.zeros_like(l.B), f.B*K2         ] ]
      cfg.Cbar = np.r_[ np.c_[ l.C               , np.zeros_like(l.C) ],
                        np.c_[ np.zeros_like(l.C), l.C                ] ]
      cfg.Qbar = np.r_[ np.c_[ Q1               , np.zeros_like(Q1)                ],
                        np.c_[ np.zeros_like(Q1), Q2+f.B*K2*f.C*P*f.C.T*K2.T*f.B.T+f.B*K2*R2*K2.T*f.B.T ] ]
      cfg.Rbar = np.r_[ np.c_[ R2               , np.zeros_like(R2) ],
                        np.c_[ np.zeros_like(R2), R2                ] ]
      # Compute the Augmented Kalman Gain
      Xbar = linalg.solve_discrete_are( cfg.Abar.T, cfg.Cbar.T, cfg.Qbar, cfg.Rbar )
      l.Lbar = Xbar * cfg.Cbar.T * linalg.inv( cfg.Cbar*Xbar*cfg.Cbar.T + cfg.Rbar )
      # Initialize Augmented Estimate
      l.xh_old_bar = np.matrix(np.zeros((8,1)))
      # Turn Off Switch
      cfg.leaderEstSwitch = 0
    
    # Calculate New Estimate
    ubar = np.r_[ l.u_old, cfg.r[:,0], np.matrix(np.zeros((2,1))) ]
    l.xh_new_bar = cfg.Abar * l.xh_old_bar + \
                   cfg.Bbar * ubar + \
                   l.Lbar   * ( np.r_[ l.y11, l.y12 ] - 
                   cfg.Cbar * cfg.Abar * l.xh_old_bar - 
                   cfg.Cbar * cfg.Bbar * ubar )
    # Update Old Estimate
    l.xh_old_bar = l.xh_new_bar
    
    xh_new = l.xh_new_bar[0:4,0]
    
  return xh_new
  
# # # # # # # # # # # # # # #
# Function:  leaderFrame    #
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


def plotError():
  """
  plotError() will plot the error in estimation:
  e = (xh - x)^T * (xh - x)
  """
  
  fig = plt.figure()
  ax = fig.add_subplot(111)
  ax.plot(np.matrix(np.arange(0.0,cfg.T,cfg.Ts)).T, cfg.e.T, \
  'b', label='$(\hat{x}-x)^T(\hat{x}-x)$')
  ax.grid()
  ax.legend(loc=2)
  plt.ylabel('Error [m]',rotation=90)
  plt.xlabel('Time [s]')
  if cfg.leaderEstType == '--simple':
    plt.title('Simple Kalman Filter')
  else:
    plt.title('Augmented Kalman Filter')
  plt.show()


# # # # # # # # # # # # #
# # # # # # # # # # # # #

if __name__ == '__main__':
  main()

# # # # # # # # # # # # #
# # # # # # # # # # # # #