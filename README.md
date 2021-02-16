# kinematic-sim

This is a kinematic simulator. Its biggest strengths are that the underlying
position and velocity representations are robust, and this implementation is
lightweight and fast. Below are high-level descriptions of the constituent
scripts and classes (for users).

## simulate.py
This is the workhorse of the simulator. It contains the loop that does a single
"run" of simulated bodies, including precise timing. It also has convenience
functions for loading simulation parameters from YAML, as well as a CLI.

CLI parameters:
- `config`: Input config YAML file, defining simulated Things
- `time`: Total simulation execution time, in seconds
- `speed`: Simulation speed; a speed of 2, for example, makes the simulation
twice as fast.
- `output`: Output file, where metrics may be written

## thing.py (Thing Class)
The Thing class is the representation of bodies. All objects in the simulator
are represented by "thing" objects. This class should be subclassed to create
more complex behavior.

The member variables are:
- `name`: String. Use is up to the developer, but I suggest using it to identify
homogeneous groups/types of things.
- `position`: Transform. The position (translation, rotation) of the thing.
- `velocity`: Transform. The velocity (translation, rotation) of the thing.
Applied to the position at each update
> Warning: Be careful when combining translations and rotations! They might not
have the effects that you think.
- `radius`: Float. The radius of the sphere defining the extents of the thing.
- `vote_exit`: Boolean. The simulator automatically exits when all things agree
that it should.
- `force_exit`: Boolean. Any thing can force the simulation to end immediately.
- `metrics`: Dict. Anything saved as a "metric" will be output at the end of
the simulation.

The member functions are:
- `update(delta, others)`: Called every frame. Includes the time since this
thing last excecuted, as well as all other things in the world to operate on.
> You should be putting most of your code in the update function for subclasses.
- `is_colliding(other)`: Convenience function for collision 
- `from_dict(properties)`: A static method to instantiate a Thing of this type
from a dictionary (effectively, YAML configuration).
> Don't forget to implement `from_dict` when subclassing Thing. In addition,
ensure that your subclass is recorded in `simulate.py`'s `THINGS` member list.

## transform.py (Transform Class)
The Transform class is the backbone of the simulated kinematics. Formally, it's
a representation of the 3D Special Euclidean group with the General Linear
group. It has convenience member functions, as well as some static helpers.

Most developers should only need to use the convenience member functions:
- `__init__(is_velocity, pos, rot)`: Constructs a transform using a position,
rotation, and whether it encodes a velocity (which has a different
representation)
- `displace(other)`: Displaces _this_ transform by the transformation encoded by
_other_
- `flow(time)`: If this transform encodes a velocity, then this function travels
along that velocity for the provided unit time.
- `set_pos, get_pos, set_rot, get_rot`: Getters and setters for positions and
rotations.
> Use the getters and setters! They make things faster, and you probably
shouldn't interface with the member variables directly