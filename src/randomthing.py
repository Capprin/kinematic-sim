# randomthing.py
# Base Thing, with a randomly defined position

import numpy as np
from thing import Thing
from transform import Transform

class RandomThing(Thing):

  NAME='random'

  def __init__(self, minTrans=[0]*3, maxTrans=[10]*3, vel=[0]*6, radius=0.25):
    pos = np.random.uniform(minTrans, maxTrans, 3)
    trans = np.pad(pos, (0,3), 'constant')
    super().__init__(trans, vel, radius)
    self.metrics['init'] = trans.tolist() #make sure yaml-readable

  @staticmethod
  def from_dict(properties):
    if properties is None:
      return RandomThing()
    minTrans = properties['min'] if 'min' in properties else [0]*3
    maxTrans = properties['max'] if 'max' in properties else [10]*3
    vel = properties['vel'] if 'vel' in properties else [0]*6
    radius = properties['radius'] if 'radius' in properties else 0.25
    return RandomThing(minTrans, maxTrans, vel, radius)