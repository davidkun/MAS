#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time, random
import agent


def main():
  """
  This is a testing function to make sure
  the code is running properly.
  
  Run this to check for errors.
  """
  
  point1 = agent.point(np.matrix([[1.0],[1.0],[0.2],[0.3]]))
  
  runSim(point1)




def runSim(vid):
  """
  Simulate vehicle movement in the xy-plane.
  """
  fig = plt.figure()
  ax = fig.add_subplot(111, \
  autoscale_on=False, \
  xlim=(0, 20), ylim=(0, 20))
  plt.ion()
  plt.hold(True)
  abc = ax.plot(vid.X[0,0], vid.X[1,0], 'o')
  
  for i in range(10):
    plt.setp(abc[0], data=(vid.X[0,0],vid.X[1,0]))
    plt.draw()
    #time.sleep(1)
    vid.f(u=np.random.rand(2,1))
  
  plt.ioff()
  track = ax.plot(vid.Xhist[:,0],vid.Xhist[:,1])
  plt.setp(track, linestyle=':', color='b')
  plt.show()


if __name__ == '__main__':
  main()

