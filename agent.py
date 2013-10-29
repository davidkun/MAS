#!/usr/bin/env python
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
  
  def __init__( self, X=np.zeros((4,1)) ):
    """ 
    Initialize point dynamics 
    """
    
    self.X0 = np.matrix(X)
    self.X = np.matrix(X)
    
    self.Xhist = np.matrix([self.X[0,0],self.X[1,0]])
    
    self.A = np.matrix([ [1, 0, Ts, 0 ],
                         [0, 1, 0 , Ts],
                         [0, 0, 1 , 0 ],
                         [0, 0, 0 , 1 ] ])
  
    self.B = np.matrix([ [Ts**2/2, 0        ],
                         [0      , Ts**2/2  ],
                         [Ts     , 0        ],
                         [0      , Ts       ]])

    self.C = np.asmatrix(np.identity(4))
    
    
  def step( self, u=np.zeros((2,1)) ):
    """
    Step in time (integrate)
    """
    
    # State Noise
    w = np.matrix(np.random.multivariate_normal(stateMean,stateCov)).T
    
    # State
    self.X = self.A*self.X + self.B*u + w
    
    # Update History of X Position (for plotting)
    self.Xhist = np.r_[self.Xhist, np.matrix([self.X[0,0],self.X[1,0]])]
  
  
  def observe( self ):
    """
    Return noisy output measurement to simulate sensor error.
    """
    
    # Sensor Noise
    v = np.matrix(np.random.multivariate_normal(sensorMean,sensorCov)).T
    
    # Measurable Output
    self.Y = self.C*self.X + v
    
    return self.Y
  