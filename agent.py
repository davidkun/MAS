#!/usr/bin/env python
import math
import numpy as np
from cfg import *

class point():
  """ 
  'point' represents a point-mass agent in a multi-vehicle system (MAS).
  
  The agent is dynamic, it reads location transmissions,
  and has the ability to communicate its own state, X, where:
  
  X = [ x ]     X[k+1] = A * X[k]  +  B * u[k]  +  w[k]
      | y |     
      | x'|     Y[k]   = C * X[k]  +  v[k]
      [ y']
  
  where w[k], v[k] are indpendent random vectors, 
  with zero mean, constant covariance, normal distributions.
  """
  
  def __init__( self, 
                X = np.zeros((4,1)),
                stateCov   = np.identity(4) * 0.0001,
                sensorCov1 = np.identity(4) * 0.0001,
                sensorCov2 = np.identity(4) * 0.0001 ):
    """ 
    Initialize point dynamics 
    """
    
    self.X0    = np.matrix(X)
    self.X     = np.matrix(X)
    self.Xhist = np.matrix([self.X[0,0],self.X[1,0]])
    
    self.A     = np.matrix([ [1, 0, Ts, 0 ],
                             [0, 1, 0 , Ts],
                             [0, 0, 1 , 0 ],
                             [0, 0, 0 , 1 ] ])
  
    self.B     = np.matrix([ [0.5*Ts**2, 0.        ],
                             [0.       , 0.5*Ts**2 ],
                             [Ts       , 0.        ],
                             [0.       , Ts        ]])

    self.C = np.matrix(np.identity(4))
    
    # Mean and Covariance of Noise in System
    self.stateMean  = (0,0,0,0)
    self.stateCov   = stateCov  # Q
    self.sensorMean = (0,0,0,0)
    self.sensorCov1  = sensorCov1  # R11 or R22
    self.sensorCov2  = sensorCov2  # R12 or R21
    
    # Initialize State Estimate and Control Input
    self.xh_old = np.matrix(np.zeros((4,1)))
    self.u_old  = np.matrix(np.zeros((2,1)))
    self.Xref_new   = np.matrix(np.zeros((4,1)))
    self.Xref_old   = np.matrix(np.zeros((4,1)))
    
    
  def step( self, u=0 ):
    """
    Step in time (integrate)
    """
    
    # State Noise
    w = np.matrix(np.random.multivariate_normal(self.stateMean,self.stateCov)).T
    
    # State
    self.X = self.A*self.X + self.B*u + w
    
    # Update History of X Position (for plotting)
    self.Xhist = np.r_[self.Xhist, np.matrix([self.X[0,0],self.X[1,0]])]
  
  
  def observe( self ):
    """
    Return noisy output measurement to sensing agent.
    """
    
    # Sensor Noise
    v = np.matrix(np.random.multivariate_normal(self.sensorMean,self.sensorCov2)).T
    
    # Measurable Output
    Y = self.C*self.X + v
    
    return Y
  
  
  def selfObserve( self ):
    """
    Return noisy output measurement to itself.
    """
    
    # Sensor Noise
    v = np.matrix(np.random.multivariate_normal(self.sensorMean,self.sensorCov1)).T
    
    # Measurable Output
    Y = self.C*self.X + v
    
    return Y
  
