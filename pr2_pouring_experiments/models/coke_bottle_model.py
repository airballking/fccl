#!/usr/bin/python

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from math import *

rospy.init_node('pancake_bottle_model_simple')
pub = rospy.Publisher('visualization_marker', Marker)

# default: /l_gripper_tool_frame
import sys
frame_name = sys.argv[1]

rate = rospy.Rate(2)
counter_id = 0
while not rospy.is_shutdown():

  m = Marker()

  m.id = counter_id
  m.ns = frame_name
  m.action = m.ADD
  m.type = m.MESH_RESOURCE

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = frame_name
  m.color = ColorRGBA(255.0/255.0, 224.0/255.00, 150.0/255.0, 1)
  m.lifetime = rospy.Duration(1)

  m.mesh_resource = "package://pr2_pouring_experiments/models/Coke-Bottle.dae";
  m.scale = Vector3(0.0254,0.0254,0.0254)
  
  m.mesh_use_embedded_materials = True
  m.pose.position = Point(0, 0, 0)
  m.pose.orientation.w = 1

  m.frame_locked = True

  pub.publish(m)
  rate.sleep()
