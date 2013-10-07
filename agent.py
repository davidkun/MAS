#!/usr/bin/env python
import numpy as np


class point():
  """ Represents a UAV in a multi-vehicle system (MAS).
  
  The agent is dynamic, it reads location transmissions,
  and has the ability to communicate its own state.
  """
  
  def __init__( self, X=np.zeros((4,1)), Ts=1.0 ):
    
    self.X = X
    self.Xhist = np.array([[self.X[0,0],self.X[1,0]]])
    
    self.A = np.matrix([ [1,0,Ts,0],
                         [0,1,0,Ts],
                         [0,0,0,0],
                         [0,0,0,0] ])
  
    self.B = np.matrix([ [0,0],
                         [0,0],
                         [1,0],
                         [0,1] ])
  
  def f(self, u=np.zeros((2,1))):
    self.X = self.A*self.X + self.B*u
    L = np.shape(self.Xhist)[0]
    x,y = self.X[0,0], self.X[1,0]
    self.Xhist = np.insert(self.Xhist, L, [x,y], axis=0)
