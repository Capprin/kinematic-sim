# thing.py
# represents agents in the world

# TODO: subclass for random positions, random seeds, more/less outputs

# TODO: currently assumes sphere shapes, to simplify collisions
# if you want more shapes:
    # implement the shape stub
    # update __init__ to take a shape instead of a radius
    # update is_colliding to do coordinate transformation and shape overlap

from transform import Transform
from numpy.linalg import norm

class Thing(object):

  # static name, for type indexing
  NAME = 'thing'

  def __init__(self, trans=[0]*6, vel=[0]*6, radius=0.25):
    # cosmetic name, specific to this instance
    self.name = ''
    # set spatial member vars
    self.position = Transform(is_velocity=False, pos=trans[:3], rot=trans[3:])
    self.velocity = Transform(is_velocity=True, pos=vel[:3], rot=vel[3:])
    self.radius = 0.25
    # set simulation parameters
    self.vote_exit = False
    self.force_exit = False
    # metrics; add to this for output
    self.metrics = dict()

  # update myself in the world. all other things are passed, for convenience
  # this should be extended!
  def update(self, delta, others):
    self.position.displace(self.velocity.flow(delta))

  # test for collision
  def is_colliding(self, other):
    return norm(self.position.get_pos() - other.position.get_pos()) < self.radius + other.radius

  # return an instance of myself from a dictionary
  # subclass for more specific behavior
  @staticmethod
  def from_dict(properties, previous_things=None):
    if properties is None:
      return Thing()
    trans = properties['trans'] if 'trans' in properties else [0]*6
    vel = properties['vel'] if 'vel' in properties else [0]*6
    rad = properties['rad'] if 'rad' in properties else 0.25
    return Thing(trans, vel, rad)