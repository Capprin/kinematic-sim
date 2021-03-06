# rrtthing.py
# priority queue RRT implementation

import numpy as np
from thing import Thing
from randomthing import RandomThing
from transform import Transform

class RRTThing(RandomThing):

  NAME = 'rrt'

  # TODO: this only supports one goal; add support for a _specific_ goal

  def __init__(self, minTrans=[0]*3, maxTrans=[10, 10, 0], vel=[0]*6, radius=0.25,
               max_iterations=100):
    # do standard initialization
    super().__init__(minTrans, maxTrans, vel, radius)
    
    

  def update(self, delta, others):
    # TODO: build RRT tree on first iteration; save path
    # TODO: report as metrics:
        # planned path (or failure to plan path)
            # could also include length
        # initialization time

    # keep track of position along path
    # ea. update, advance position:
        # move with provided velocity (max vel) along ea. segment
        # if will overshoot next node, set position to node and rotate
        # continue until at goal
    # once at goal:
        # vote exit
    pass

  def build_rrt_tree(self, others, bounds, max_iterations, step_length):
    # intitialize tree to store configurations
    # list of tuples: (Transform, ParentIndex)
    start = Thing(radius=self.radius)
    start.position = self.position
    start.velocity = self.velocity
    tree = [(start, -1)]
    for i in range(max_iterations):
        # generate random free configuration
        # TODO: if runtimes are bad, sample goal occasionally (with eps)
        q_rand = RandomThing(minTrans=bounds[0], maxTrans=bounds[1], radius=self.radius)
        for other in others:
          if other.name != 'goal' and q_rand.is_colliding(other):
            # skip any colliding configurations
            continue
        # traverse tree; find closest configuration (by position)
        q_near = tree[0][0]
        near_idx = 0
        nearest_dist = np.linalg.norm(q_near.position - q_rand.position)
        for idx, node in enumerate(tree):
          dist = np.linalg.norm(node[0].position - q_rand.position)
          if dist < nearest_dist:
            q_near = node[0]
            near_idx = idx
            nearest_dist = dist
        # create new node
        # traverse by step_length from q_near along line towards q_rand
        q_new = q_near
        disp = q_new.position.displacement_to(q_rand)
        disp.set_pos(np.zeros(3)) #toss displacement; we're using our own
        q_new.position = q_new.position.displace(disp) # rotate towards q_rand
        q_new.position = q_new.position.displace(step_length*Transform.FORWARD)
        # ensure new position isn't in collision
        # this ignores points along the path; I'm kinda assuming step length is
        # smaller than the radius, so that shouldn't matter
        # also: getting goal now, for comparison later
        goal = None
        for other in others:
            if other.name == 'goal':
              goal = other
            elif q_new.is_colliding(other):
              continue
        # add new node to tree
        tree.append((q_new, near_idx))
        # if new node is goal, done building tree
        if q_new.is_colliding(goal):
          # construct path by traversing from goal -> start on tree (then reverse)
          path = [tree[-1][0]]
          next_idx = tree[-1][1] #start at end of list
          while next_idx != -1: #go until starting element
            path.append(tree[next_idx][0])
            next_idx = tree[next_idx][1]
          path.reverse()
          # place dummy Things along path for sequential planners
          # this might fuck with Thing iteration in simulate.py; check that out
          for thing in path:
            others.append(Thing)
          return path

  @staticmethod
  def from_dict(properties):
    pass