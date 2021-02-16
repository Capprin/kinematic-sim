# shape.py
# represents the extents of a body

# TODO: this is a stub. if you want more than spheres, implement this, with:
    # transform to store position, orientation
    # contains function for individual points
    # overlap function between myself and other shapes (in my frame)
    # draw function, for the brave

class Shape:

  # defines actual shape
  def contains(self, pos):
    pass

  # effectively does collision detection
  def overlaps(self, other):
    pass