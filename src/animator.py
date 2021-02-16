# animator.py
# wrapper for animated 3D scatter in the context of simulations

import numpy as np
import matplotlib.pyplot as plt

# create figure for updates
def init():
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.autoscale(enable=True, tight=False)
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.ion()
  plt.show()
  return ax

# hook to update figure
def update(ax, things):
  plt.cla()
  # collect data (these must be different because of list mutability)
  x = [0]*len(things)
  y = [0]*len(things)
  z = [0]*len(things)
  s = [0]*len(things)
  for i, thing in enumerate(things):
    pos_vec = thing.position.get_pos()
    # should be looped, but not verbose
    x[i] = pos_vec[0]
    y[i] = pos_vec[1]
    z[i] = pos_vec[2]
    s[i] = np.pi * (10*thing.radius) ** 2
  ax.scatter(x, y, z, s=s, c='blue')
  plt.draw()
  plt.pause(0.01)

def end():
  plt.ioff()
  plt.show()