#!/usr/bin/env python
import numpy as np
from cfg import *

class point():
  """ 
  'point' represents a point-mass agent in a multi-vehicle system (MAS).
  
  The agent is dynamic, it reads location transmissions,
  and has the ability to communicate its own state, X, where:
  
  X = [ x ]     X[k+1] = A * X[k]  +  B * u[k]
      | y |     
      | x'|     Y[k]   = C * X[k]
      [ y']
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
    
    #self.Z = np.zeros([4,1])
    
    
  def step( self, u=np.zeros((2,1)) ):
    """
    Step in time (integrate)
    """
    
    # State
    self.X = self.A*self.X + self.B*u

    # Measurable Output
    self.Y = self.C*self.X
    
    # Update History of X Position (for plotting)
    self.Xhist = np.r_[self.Xhist, np.matrix([self.X[0,0],self.X[1,0]])]
  