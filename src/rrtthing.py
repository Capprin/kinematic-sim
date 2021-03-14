# rrtthing.py
# priority queue RRT implementation

import numpy as np
from copy import deepcopy
from time import time_ns
from thing import Thing
from randomthing import RandomThing
from transform import Transform

class RRTThing(RandomThing):

  NAME = 'rrt'
  EPS = 0.5
  STEP_LENGTH = 0.5

  def __init__(self, goal, minTrans=[0]*3, maxTrans=[10, 10, 0],
               searchMin=[0]*3, searchMax=[10,10,0], vel=Transform.FORWARD,
               radius=0.25, max_iterations=100):
    # do standard initialization
    super().__init__(minTrans, maxTrans, vel, radius)
    # save relevant RRT data
    self.goal = goal
    self.bounds = (searchMin, searchMax)
    self.max_iterations = max_iterations
    self.step = 0

  # create rrt tree offline
  def on_start(self, others):
    start_ns = time_ns()
    self.path = self.build_rrt_tree(others)
    init_time_s = (time_ns() - start_ns)/(10**9)
    # if no path, force exit now
    if not self.path:
      self.force_exit = True
    
    # report rrt info as metrics
    self.metrics['init_time_s'] = init_time_s
    if self.path:
      path_length = RRTThing.STEP_LENGTH*(len(self.path)-1)
      self.metrics['path_length'] = path_length
    self.metrics['found_goal'] = False

  # navigate generated path
  def update(self, delta, others):
    # check for goal collision
    if self.is_colliding(self.goal):
      self.metrics['found_goal'] = True
      self.vote_exit = True
      return #no longer need to move
    # estimate if we'll overshoot node
    abs_vel = np.linalg.norm(self.velocity.get_pos())
    disp = self.position.displacement_to(self.path[self.step+1].position)
    remaining_dist = np.linalg.norm(disp.get_pos())
    # use time to estimate overshoot; Transform.flow() could be better
    if remaining_dist/abs_vel < delta:
      # we'll probably overshoot next node; update step
      self.step += 1
      # set position to next node
      self.position = deepcopy(self.path[self.step].position) #update position
    else:
      # step along path
      disp_step = Transform(pos=abs_vel*delta*disp.get_pos()/remaining_dist)
      self.position.displace(disp_step)

  # creates RRT tree; returns path to goal
  def build_rrt_tree(self, others):
    # intitialize tree to store configurations
    # list of tuples: (Transform, ParentIndex)
    start = Thing(radius=self.radius)
    start.position = deepcopy(self.position)
    tree = [(start, -1)]
    for _ in range(self.max_iterations):
      # generate random free configuration (or occasionally sample goal)
      q_rand = None
      if np.random.uniform() > RRTThing.EPS:
        q_rand = RandomThing(minTrans=self.bounds[0], maxTrans=self.bounds[1],
                             radius=self.radius)
      else:
        q_rand = self.goal
      for other in others:
        if other.name != self.goal.name and q_rand.is_colliding(other):
          # skip any colliding configurations
          continue
      # traverse tree; find closest configuration (by position)
      q_near = tree[0][0]
      near_idx = 0
      nearest_dist = np.linalg.norm(q_near.position.get_pos() - q_rand.position.get_pos())
      for idx, node in enumerate(tree):
        dist = np.linalg.norm(node[0].position.get_pos() - q_rand.position.get_pos())
        if dist < nearest_dist:
          q_near = node[0]
          near_idx = idx
          nearest_dist = dist
      # create new node
      # traverse by step_length from q_near along line towards q_rand
      q_tmp = deepcopy(q_near)
      disp_full = q_tmp.position.displacement_to(q_rand.position)
      disp_radius = Transform(pos=self.radius*disp_full.get_pos()/np.linalg.norm(disp_full.get_pos()))
      # ensure ea. location along path isn't in collision
      for _ in np.arange(0, RRTThing.STEP_LENGTH, self.radius):
        q_tmp.position.displace(disp_radius)
        for other in others:
          if other.name != self.goal.name and q_tmp.is_colliding(other):
            continue
      # add new configuration
      q_new = deepcopy(q_tmp)
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
        for i in range(len(path)-1):
          disp_full = path[i].position.displacement_to(path[i+1].position)
          disp_radius = Transform(pos=self.radius*disp_full.get_pos()/np.linalg.norm(disp_full.get_pos()))
          q_tmp = deepcopy(path[i].position)
          for _ in np.arange(0, RRTThing.STEP_LENGTH, self.radius):
            q_tmp.displace(disp_radius)
            new_thing = Thing()
            new_thing.vote_exit = True
            new_thing.position = deepcopy(q_tmp)
            others.append(new_thing)
        return path

  @staticmethod
  def from_dict(properties, previous_things):
    if properties is None or previous_things is None:
      raise Exception('rrtthing: properties and previous_things must be supplied')
    # text properties
    minTrans = properties['min'] if 'min' in properties else [0]*3
    maxTrans = properties['max'] if 'max' in properties else [10, 10, 0]
    searchMin = properties['search_min'] if 'search_min' in properties else [0]*3
    searchMax = properties['search_max'] if 'search_max' in properties else [10, 10, 0]
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
    return RRTThing(goal, minTrans, maxTrans, searchMin, searchMax, vel, radius, max_iterations)