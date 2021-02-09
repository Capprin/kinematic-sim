# transform.py
# manages both lie group and lie algebra elements in SE(3)
# this will probably come back to bite me

import numpy as np

# TODO: add tests if you gotta
class Transform:

  def __init__(self, is_velocity, pos=np.array([0.,0.,0.]), rot=np.array([0.,0.,0.])):
    self._is_velocity = is_velocity
    rot_mat = None
    if is_velocity:
      rot_mat = Transform.inst_rot_mat_3d(rot)
    else:
      rot_mat = Transform.rot_mat_3d(rot)
    self.matrix = np.block([[rot_mat, pos],[0., 0., 0., (float)(not self._is_velocity)]])
    self._pos = pos
    self._rot = rot
  
  def displace(self, other):
    if self._is_velocity:
      raise Exception("Transform: don't multiply velocities; it causes problems")
    self.matrix = self.matrix @ other.matrix
    self._pos = self.matrix[:2,3]
    self._rot = Transform.rot_vec(self.matrix[:2,:2])

  def get_pos(self):
    return self._pos

  def set_pos(self, pos):
    self.matrix[:2,3] = pos
    self._pos = pos

  def get_rot(self):
    return self._rot

  def set_rot(self, rot):
    self.matrix[:2,:2] = Transform.rot_mat_3d(rot)
    self._rot = rot

  @staticmethod
  # produces a 2D rotation matrix
  def rot_mat_2d(axis, theta):
    if axis=='x':
      return np.array([[1., 0., 0.],
                       [0., np.cos(theta), -np.sin(theta)],
                       [0., np.sin(theta), np.cos(theta)]])
    elif axis=='y':
      return np.array([[np.cos(theta), 0., np.sin(theta)],
                       [0., 1., 0.],
                       [-np.sin(theta), 0., np.cos(theta)]])
    elif axis=='z':
      return np.array([[np.cos(theta), -np.sin(theta), 0.],
                       [np.sin(theta), np.cos(theta), 0.],
                       [0., 0., 1.]])

  @staticmethod
  # produces a 3D rotation matrix (in YPR order)
  def rot_mat_3d(rot):
    return Transform.rot_mat_2d('z', rot[2]) @ Transform.rot_mat_2d('y', rot[1]) @ Transform.rot_mat_2d('x', rot[0])

  @staticmethod
  # produces a 3D rotation matrix for the Lie Algebra
  def inst_rot_mat_3d(rot):
    return np.array([[0., -rot[2], rot[1]], [rot[2], 0., -rot[0]], [-rot[2], rot[0], 0]])

  @staticmethod
  # gets non-unique rotations producing a rotation matrix
  # method from https://www.gregslabaugh.net/publications/euler.pdf
  def rot_vec(rot_mat):
    rot = np.array([0., 0., 0.])
    if np.abs(rot_mat[2,0]) != 1:
      rot[1] = -np.arcsin(rot_mat[2,0]) #another possible that's ignored
      cosy = np.cos(rot[1])
      rot[0] = np.arctan2(rot_mat[2,1]/cosy, rot_mat[2,2]/cosy)
      rot[2] = np.arctan2(rot_mat[1,0]/cosy, rot_mat[0,0]/cosy)
    else:
      rot[2] = 0
      if rot_mat[2,0] == -1:
        rot[1] = np.pi/2
        rot[0] = rot[2] + np.arctan2(rot_mat[0,1], rot_mat[0,2])
      else:
        rot[1] = -np.pi/2
        rot[0] = -rot[2] + np.arctan2(rot_mat[0,1], rot_mat[0,2])

