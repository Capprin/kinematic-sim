# simulate.py
# this is the workhorse of the simulator. it contains the loop and all
# input/output options.

# TODO: wrap in another script to chain multiple simulations
# for that script, you'll need to est. more configuration than things
    # need equivalent starting conditions, etc

import argparse
from time import time, time_ns
from tqdm import tqdm
import yaml

# things
from thing import Thing

# keep track of all types of things here
THINGS = {Thing.NAME:Thing}

# run a simulation
def simulate(things, max_time_s, speed=1.0):
  # keep track of time
  start_s = time()
  start_ns = time()
  elapsed_s = 0
  elapsed_ns = [0]*len(things) #ea. thing gets its own delta

  # simulator loop
  # TODO: fix time dilation in output
  done = False
  progress = tqdm(desc='Simulation Progress', total=max_time_s)
  while elapsed_s < max_time_s and not done:
    exit_vote = 0
    # update each thing
    for i,thing in enumerate(things):
      # pass time since last update for each
      delta = speed*(time_ns() - elapsed_ns[i])
      thing.update(delta, things)
      elapsed_ns[i] = time_ns() - start_ns
      # check for exit
      if thing.force_exit:
        done = True
        break
      exit_vote += int(thing.vote_exit)
    progress.update(speed*(time()-elapsed_s))
    elapsed_s = speed*(time() - start_s)
    if exit_vote >= len(things):
      done = True
  progress.close()

  # output metrics
  metrics = dict()
  for i, thing in enumerate(things):
    if thing.metrics:
      metrics[str(i) + thing.NAME] = thing.metrics
  return metrics

# read supplied things yaml (stream, string, or file), returning thing list
# all behavior (randomness, etc) ought to be defined as a thing
def read_things(yaml_in):
  thing_dict = yaml.safe_load(yaml_in)
  thing_list = []
  # est. config as list of things (with properties)
  for conf_name, conf_properties in thing_dict.items():
    # identify thing based on name
    for thing_name, thing_class in THINGS.items():
      if thing_name in conf_name:
        thing_list.append(thing_class.from_dict(conf_properties))
  return thing_list

if __name__=="__main__":
  parser = argparse.ArgumentParser(description='Runs single simulation based on configuration')
  parser.add_argument('config', metavar='config', type=str, help='Configuration YAML file')
  parser.add_argument('-t', '--time', default=60, type=int, help='Simulation time in seconds (default 60s)')
  parser.add_argument('-s', '--speed', default=1., type=float, help='Simulation speed multiplier (default 1.0f)')
  parser.add_argument('-o', '--output', type=str, help='Output file. If unspecified, output written to console')
  args = parser.parse_args()
  with open(args.config,'r') as yf:
    things = read_things(yf)
  print('Running simulation...')
  res = simulate(things, args.time, args.speed)
  if args.output is not None:
    with open(args.output, 'w') as of:
      yaml.dump(res, of)
  else:
    print('Returned Metrics:')
    print(res)