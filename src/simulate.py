# simulate.py
# this is the workhorse of the simulator. it contains the loop and all
# input/output options.

# TODO: wrap in another script to chain multiple simulations
# for that script, you'll need to est. more configuration than things
    # need equivalent starting conditions, etc

import argparse
import animator as anim
from time import time, time_ns
from tqdm import tqdm
import yaml

# things
from thing import Thing
from randomthing import RandomThing
from rrtthing import RRTThing
from obstaclething import ObstacleThing

# keep track of all types of things here
THINGS = [Thing, RandomThing, ObstacleThing, RRTThing]

# run a simulation
def simulate(things, max_time_s, speed=1.0, animate=True):
  # run on_start, so Things can do any req. initialization
  print('Performing any required initialization...')
  i = 0
  num_things = len(things)
  done = False
  init_progress = tqdm('Initialization Progress', total=num_things, miniters=0, mininterval=1)
  while i < num_things:
    things[i].on_start(things)
    # check for force exit now
    if things[i].force_exit:
      done = True
      break
    # handle length changes
    if len(things) > num_things:
      num_things = len(things)
      init_progress.total = num_things
    i += 1
    init_progress.update(i)
  init_progress.close()
  print('Done.')

  # set up animation, draw initial state
  if animate and not done:
    axes = anim.init()
    anim.update(axes, things)

  # keep track of time
  start_s = time()
  start_ns = time_ns()
  elapsed_s = 0
  elapsed_ns = [0]*len(things) #ea. thing gets its own delta

  # simulator loop
  progress = tqdm(desc='Simulation Progress', total=max_time_s)
  while elapsed_s < max_time_s and not done:
    exit_vote = 0
    # update each thing
    num_things = len(things)
    i = 0
    while i < num_things:
      thing = things[i]
      # pass time since last update for each
      delta = speed*(time_ns() - start_ns - elapsed_ns[i])*10**(-9)
      thing.update(delta, things)
      elapsed_ns[i] = time_ns() - start_ns
      # check for exit
      if thing.force_exit:
        print('\nReceived force exit from ' + thing.name)
        done = True
        break
      exit_vote += int(thing.vote_exit)
      # update loop vars, in case more things were added
      # this is generally bad practice; done in case Things append to list
      if len(things) > num_things:
        elapsed_ns.extend([time_ns()-start_ns] * (len(things)-num_things))
        num_things = len(things)
      i += 1
    # draw updates
    if animate:
      anim.update(axes, list(reversed(things)))
    progress.update(speed*(time()-elapsed_s))
    elapsed_s = speed*(time() - start_s)
    if exit_vote >= len(things):
      done = True
  progress.close()

  # display simulation results
  if animate:
    anim.end()

  # output metrics
  metrics = dict()
  for i, thing in enumerate(things):
    if thing.metrics:
      metrics[thing.name] = thing.metrics
  return metrics

# read supplied things yaml (stream, string, or file), returning thing list
# all behavior (randomness, etc) ought to be defined as a thing
def read_things(yaml_in):
  thing_dict = yaml.safe_load(yaml_in)
  thing_list = []
  # est. config as list of things (with properties)
  for conf_name, conf_properties in thing_dict.items():
    # identify thing based on name
    for thing_class in THINGS:
      if thing_class.NAME in conf_name:
        inst = thing_class.from_dict(conf_properties, thing_list)
        inst.name = conf_name
        thing_list.append(inst)
  return thing_list

if __name__=="__main__":
  parser = argparse.ArgumentParser(description='Runs single simulation based on configuration')
  parser.add_argument('config', metavar='config', type=str, help='Configuration YAML file')
  parser.add_argument('-t', '--time', default=60, type=int, help='Simulation time in seconds (default 60s)')
  parser.add_argument('-s', '--speed', default=1., type=float, help='Simulation speed multiplier (default 1.0f)')
  parser.add_argument('-o', '--output', type=str, help='Output file. If unspecified, output written to console')
  parser.add_argument('-a', '--animate', default=False, action='store_true', help='Run with animation')
  args = parser.parse_args()
  with open(args.config,'r') as yf:
    things = read_things(yf)
  print('Starting simulation')
  res = simulate(things, args.time, args.speed, args.animate)
  print('Done.')
  if args.output is not None:
    with open(args.output, 'w') as of:
      yaml.dump(res, of, default_flow_style=None)
  else:
    print('Returned Metrics:')
    print(res)