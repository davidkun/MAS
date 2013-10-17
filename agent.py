#!/usr/bin/env python
import numpy as np
from simParams import *

class point():
  """ 
  'point' represents a point-mass agent in a multi-vehicle system (MAS).
  
  The agent is dynamic, it reads location transmissions,
  and has the ability to communicate its own state, X, where:
  
  X = [ x ]
      | y |
      | x'|
      [ y']
  """
  
  def __init__( self, X=np.zeros((4,1)) ):
    """ 
    Initialize point dynamics 
    """
    
    self.X = X
    self.Xhist = np.array([[self.X[0,0],self.X[1,0]]])
    
    self.A = np.matrix([ [1,0,Ts,0 ],
                         [0,1,0 ,Ts],
                         [0,0,1 ,0 ],
                         [0,0,0 ,1 ] ])
  
    self.B = np.matrix([ [0      , 0        ],
                         [0      , 0        ],
                         [Ts**2/2, 0        ],
                         [0      , Ts**2/2] ])

    self.C = np.asmatrix(np.identity(4))
    
    self.Z = np.zeros([4,1])
    
    
  def step( self, u=np.zeros((2,1)) ):
    """
    Step in time (integrate)
    """
    
    # State
    self.X = self.A*self.X + self.B*u

    # Integral Control
    self.XI = self.Z - np.r_[wPts[target].T,np.zeros([2,1])]
    
    # Measurable Output
    self.Z = self.C*self.X
    
    
    # Update History of X Position (for plotting)
    sizeX = np.shape(self.Xhist)[0]
    xPos,yPos = self.X[0,0], self.X[1,0]
    self.Xhist = np.insert(self.Xhist, sizeX, [xPos,yPos], axis=0)
  