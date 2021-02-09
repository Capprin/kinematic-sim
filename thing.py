# thing.py
# represents agents in the world

# TODO: currently assumes sphere shapes, to simplify collisions
# if you want more shapes:
    # implement the shape stub
    # update __init__ to take a shape instead of a radius
    # update is_colliding to do coordinate transformation and shape overlap

from transform import Transform
from numpy.linalg import norm

class Thing:

  def __init__(self, trans=[0]*6, vel=[0]*6, radius=0.25):
    # set member vars
    self.position = Transform(is_velocity=False, pos=trans[:2], rot=trans[3:5])
    self.velocity = Transform(is_velocity=True, pos=vel[:2], rot=vel[3:5])
    self.radius = 0.25

  # update myself in the world.
  # this should be extended!
  def update(self, delta):
    self.position.displace(self.velocity.flow(delta))

  # test for collision
  def is_colliding(self, other):
    return norm(self.position.get_pos() - other.position.get_pos()) < self.radius + other.radius