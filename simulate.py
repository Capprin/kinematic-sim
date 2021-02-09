# simulate.py
# this is the workhorse of the simulator. it contains the loop and all
# input/output options.

# TODO: wrap in another script to chain multiple simulations
# for that script, you'll need to est. more configuration than things
    # need equivalent starting conditions, etc

from time import time, time_ns
from tqdm import tqdm

# run a simulation
def simulate(things, max_time_s, dilation=1.0):
  # keep track of time
  start_s = time()
  start_ns = time()
  elapsed_s = 0
  elapsed_ns = [0]*len(things) #ea. thing gets its own delta

  # simulator loop
  done = False
  progress = tqdm(desc='Simulation time (s)', total=max_time_s)
  while elapsed_s < max_time_s and not done:
    exit_vote = 0
    # update each thing
    for i,thing in enumerate(things):
      # pass time since last update for each
      delta = dilation*(time_ns() - elapsed_ns[i])
      thing.update(delta, things)
      elapsed_ns[i] = time_ns() - start_ns
      # check for exit
      if thing.force_exit:
        done = True
        break
      exit_vote += int(thing.vote_exit)
    elapsed_s = time() - start_s
    progress.update(elapsed_s)
    if exit_vote >= len(things):
      done = True
  progress.close()
  # TODO: add metrics on all the things, returned after the sim

# read supplied things yaml (string, or file), returning thing list
# all behavior (randomness, etc) ought to be defined as a thing
def read_things(yaml_in):
  # TODO: establish a structure for yaml
  # TODO: establish string -> thing mapping (thing-specific?)
  pass

if __name__=="__main__":
  # TODO: add CLI here
  pass