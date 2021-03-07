# rrtthing.py
# priority queue RRT implementation

import numpy as np
from time import time_ns
from thing import Thing
from randomthing import RandomThing
from transform import Transform

class RRTThing(RandomThing):

  NAME = 'rrt'

  def __init__(self, goal, minTrans=[0]*3, maxTrans=[10, 10, 0],
               vel=Transform.FORWARD, radius=0.25, max_iterations=100):
    # do standard initialization
    super().__init__(minTrans, maxTrans, vel, radius)
    # save relevant RRT data
    self.goal = goal
    self.step_length = radius
    self.bounds = (minTrans, maxTrans)
    self.max_iterations = max_iterations
    self.step = 0
    self.first_update = True

  def update(self, delta, others):
    # build RRT tree on first iteration; save path and metrics
    if self.first_update:
      start_ns = time_ns()
      self.path = self.build_rrt_tree(others)
      init_time_s = (time_ns() - start_ns)*10**9
      # if no path, vote exit now
      if not self.path:
        self.vote_exit = True
      
      # report rrt info as metrics
      self.metrics['init_time_s'] = init_time_s
      path_points = []
      for thing in self.path:
        path_points.append(thing.position.get_pos())
      self.metrics['path_points'] = path_points
      self.metrics['found_goal'] = False
      # no longer in first update
      self.first_update = False
    
    # navigate path in ea. update
    if self.vote_exit:
      # no path; do nothing
      return
    # check for goal collision
    if self.is_colliding(self.goal):
      self.metrics['found_goal'] = True
      self.vote_exit = True
      return #no longer need to move
    # estimate if we'll overshoot node
    abs_vel = np.linalg.norm(self.velocity.get_pos())
    remaining_dist = np.linalg.norm(self.position.get_pos() - 
                                    self.path[self.step+1].position.get_pos())
    # use time to estimate overshoot; Transform.flow() could be better
    if remaining_dist/abs_vel < delta:
      # we'll probably overshoot next node; update step
      self.step += 1
      # set position to next node
      rot_to = self.path[self.step].position.displacement_to(self.path[self.step+1].position)
      rot_to.set_pos(np.zeros(3)) #only want rotation to next node
      self.position = self.path[self.step].position #update position
      self.position.displace(rot_to) #rotate towards next next node
    else:
      # probably won't overshoot, so just integrate velocity
      super().update(delta, others)

  # creates RRT tree; returns path to goal
  def build_rrt_tree(self, others):
    # intitialize tree to store configurations
    # list of tuples: (Transform, ParentIndex)
    start = Thing(radius=self.radius)
    start.position = self.position
    start.velocity = self.velocity
    tree = [(start, -1)]
    for _ in range(self.max_iterations):
        # generate random free configuration
        # TODO: if runtimes are bad, sample goal occasionally (with eps)
        q_rand = RandomThing(minTrans=self.bounds[0], maxTrans=self.bounds[1], radius=self.radius)
        for other in others:
          if other.name != self.goal.name and q_rand.is_colliding(other):
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
        q_new.position = q_new.position.displace(self.step_length*Transform.FORWARD)
        # ensure new position isn't in collision
        # this ignores points along the path; I'm kinda assuming step length is
        # smaller than the radius, so that shouldn't matter
        # also: getting goal now, for comparison later
        for other in others:
            if other.name != self.goal.name and q_new.is_colliding(other):
              continue
        # add new node to tree
        tree.append((q_new, near_idx))
        # if new node is goal, done building tree
        if q_new.is_colliding(self.goal):
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
            others.append(thing)
          return path

  @staticmethod
  def from_dict(properties, previous_things):
    if properties is None or previous_things is None:
      raise Exception('rrtthing: properties and previous_things must be supplied')
    # text properties
    minTrans = properties['min'] if 'min' in properties else [0]*3
    maxTrans = properties['max'] if 'max' in properties else [10, 10, 0]
    vel = properties['vel'] if 'vel' in properties else Transform.FORWARD
    radius = properties['radius'] if 'radius' in properties else 0.25
    max_iterations = properties['max_iterations'] if 'max_iterations' in properties else 100
    # goal init
    goal_name = None
    if 'goal' in properties:
      goal_name = properties['goal']
    else:
      raise Exception('rrtthing: goal must be provided in properties')
    goal = None
    for thing in previous_things:
      if thing.name == goal_name:
        goal = thing
    if goal is None:
      raise Exception('rrtthing: goal name has no corresponding Thing')
    return RRTThing(goal, minTrans, maxTrans, vel, radius, max_iterations)