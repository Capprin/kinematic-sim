# thing.py
# represents agents in the world

from transform import Transform

class Thing:

  def __init__(self, trans=[0]*6, vel=[0]*6, shape=None):
    # set member vars
    self.position = Transform(is_velocity=False, pos=trans[:2], rot=trans[3:5])
    self.velocity = Transform(is_velocity=True, pos=vel[:2], rot=vel[3:5])
    self.shape = shape

  # update myself in the world.
  # this should be extended!
  def update(self, delta):
    self.position.displace(self.velocity.flow(delta))

  def is_colliding(self, other):
    pass