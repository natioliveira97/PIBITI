# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from fiducial_msgs/FiducialMapEntry.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class FiducialMapEntry(genpy.Message):
  _md5sum = "2dc3e2ac5967f3a2c19627a1fc1d7dcc"
  _type = "fiducial_msgs/FiducialMapEntry"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# pose of a fiducial
int32 fiducial_id
# translation
float64 x
float64 y
float64 z
# rotation
float64 rx
float64 ry
float64 rz

"""
  __slots__ = ['fiducial_id','x','y','z','rx','ry','rz']
  _slot_types = ['int32','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       fiducial_id,x,y,z,rx,ry,rz

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FiducialMapEntry, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.fiducial_id is None:
        self.fiducial_id = 0
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.rx is None:
        self.rx = 0.
      if self.ry is None:
        self.ry = 0.
      if self.rz is None:
        self.rz = 0.
    else:
      self.fiducial_id = 0
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.rx = 0.
      self.ry = 0.
      self.rz = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_i6d().pack(_x.fiducial_id, _x.x, _x.y, _x.z, _x.rx, _x.ry, _x.rz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 52
      (_x.fiducial_id, _x.x, _x.y, _x.z, _x.rx, _x.ry, _x.rz,) = _get_struct_i6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i6d().pack(_x.fiducial_id, _x.x, _x.y, _x.z, _x.rx, _x.ry, _x.rz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 52
      (_x.fiducial_id, _x.x, _x.y, _x.z, _x.rx, _x.ry, _x.rz,) = _get_struct_i6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i6d = None
def _get_struct_i6d():
    global _struct_i6d
    if _struct_i6d is None:
        _struct_i6d = struct.Struct("<i6d")
    return _struct_i6d
