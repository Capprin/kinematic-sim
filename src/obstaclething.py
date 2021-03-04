# obstacle.py
# "dead" thing, acting only as an obstacle to others

from randomthing import RandomThing

class ObstacleThing(RandomThing):
  
  NAME = 'obstacle'

  def __init__(self, minTrans=[0]*3, maxTrans=[10]*3, vel=[0]*6, radius=1):
    super().__init__(minTrans, maxTrans,vel,radius)
    # always vote exit so we don't block
    self.vote_exit = True

  @staticmethod
  def from_dict(properties):
    # abuse python's weak typing
    obs = RandomThing.from_dict(properties)
    obs.NAME = ObstacleThing.NAME
    obs.vote_exit = True
    return obs