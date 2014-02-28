#!/usr/bin/python

import roslib
roslib.load_manifest('fccl_nodes')

import rospy
import PyKDL as kdl
import threading

from fccl_msgs.msg import SingleArmMotionActionGoal,SingleArmMotionGoal,Constraint,Feature
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3,Quaternion
from std_msgs.msg import ColorRGBA

# some color definitions
grey = ColorRGBA(0.7, 0.7, 0.7, 0.5)
red = ColorRGBA(0.7, 0.1, 0.1, 1.0)
yellow = ColorRGBA(0.7, 0.7, 0.1, 0.6)

def marker_base(feature, config):
  m = Marker()
  m.id = config['marker_id']
  m.ns = config['ns']
  m.color = config['color']
  m.lifetime = config['lifetime']
  m.action = Marker.ADD
  m.header.frame_id = feature.reference.data
  m.header.stamp = rospy.Time.now()

  m.pose.position = feature.position
  
  return m 

def marker_point(feature, config):
  m = marker_base(feature, config)

  m.scale.x = config['point_width']
  m.scale.y = config['point_width']
  m.scale.z = config['point_width']

  m.type = Marker.SPHERE

  return m

def marker_line(feature, config):
  m = marker_base(feature, config)

  m.pose.orientation = Quaternion(*orientation_feature(feature))

  m.scale.x = config['line_width']
  m.scale.y = config['line_width']
  m.scale.z = config['line_length']

  m.type = Marker.CYLINDER

  return m

def marker_plane(feature, config):
  m = marker_line(feature, config)

  m.scale.x = config['plane_width']
  m.scale.y = config['plane_width']
  m.scale.z = config['plane_length']

  return m

def msg2vec(v):
  """Convert a geometry_msgs/Vector3 to a KDL Vector."""
  return kdl.Vector(v.x, v.y, v.z)

def orientation_feature(feature):
  eps = 1e-10 # some small number for alignment test

  direction = msg2vec(feature.direction)

  axis = kdl.Vector(0,0,1) * direction
  angle = direction.z()

  laxis = axis.Norm()
  if laxis > eps and scale > eps:
    l = sqrt((1 - angle / scale) / 2) / laxis
    qu = sqrt((1 + angle / scale) / 2)
    q = [axis.x()*l, axis.y()*l, axis.z()*l, qu]
  else:
    # aligned with x/z-axis or zero-length: no rotation needed
    q = [0, 0, 0, 1]

  return q

def extract_features(constraints):
  features = {}
  for constraint in constraints:
    features[constraint.tool_feature.name] = constraint.tool_feature
    features[constraint.object_feature.name] = constraint.object_feature

  return features

def publish_marker(publisher, feature, config):
  if feature.type.data == Feature.POINT:
    publisher.publish(marker_point(feature, config))
  if feature.type.data == Feature.LINE:
    publisher.publish(marker_line(feature, config))
  if feature.type.data == Feature.PLANE:
    publisher.publish(marker_plane(feature, config))

class FeatureVisualization:

  def __init__(self, config):
    self.mylock = threading.Lock()
    self.features = {}
    self.config = config
    self.publisher = rospy.Publisher('visualization_marker', Marker)

  def set_constraints(self, constraints):
    self.mylock.acquire()

    self.features = {}
    for constraint in constraints:
      self.features[constraint.tool_feature.name] = constraint.tool_feature
      self.features[constraint.object_feature.name] = constraint.object_feature

    self.mylock.release()

  def publish_markers(self):
    self.mylock.acquire()

    self.config['marker_id'] = 0
    for feature in self.features:
      publish_marker(self.publisher, self.features[feature], config)
      config['marker_id'] += 1

    self.mylock.release()

def callback(msg):
  feature_visualization.set_constraints(msg.goal.constraints)

if __name__ == "__main__":
  
  rospy.init_node('feature_visualization')

  rate = rospy.Rate(20)

  config = {'ns': 'features',
            'lifetime': rospy.Duration(0.5),
            'color': yellow,
            'point_width': 0.02,
            'line_width': 0.02,
            'line_length': 0.05,
            'plane_width': 0.05,
            'plane_length': 0.02}
 
  feature_visualization = FeatureVisualization(config)

#  subscriber = rospy.Subscriber('~constraint_goal', SingleArmMotionGoal, callback)
  subscriber = rospy.Subscriber('/l_arm_fccl_controller/command/goal', SingleArmMotionActionGoal, callback)

  while not rospy.is_shutdown():
    feature_visualization.publish_markers()
    rate.sleep()
