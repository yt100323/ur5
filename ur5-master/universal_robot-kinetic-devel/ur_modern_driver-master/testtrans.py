#!/usr/bin/env python
import sys
import copy
import rospy,sys
import moveit_commander
import actionlib
import roslib; roslib.load_manifest('ur_modern_driver')
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

from geometry_msgs.msg import  PoseStamped,Pose,PointStamped
#import roslib; roslib.load_manifest(PKG)
import unittest

import tf2_py as tf2
import tf2_ros
import tf2_geometry_msgs
#from geometry_msgs.msg import PointStamped
import tf2_kdl
import PyKDL


def test_convert():
    p1 = PointStamped()
    p1.header.frame_id = "base_link"
    p1.header.stamp = rospy.Time(0.0)
    p1.point.x = 0.0
    p1.point.y = 0.0
    p1.point.z = 0.0
    try:
       try:
           now = rospy.Time.now()
           listener.waitForTransform("/base_link", "/kinect_link", now, rospy.Duration(4.0))
           (trans,rot) = listener.lookupTransform("/base_link", "/kinect_link", now)
    
    except (tf.LookupException, tf.ConnectivityException):

        p2 = t.transform(p1, "kinect2_link")
        rospy.loginfo("p1: %s, p2: %s" % (p1, p2))
    except tf2.TransformException as e:
            rospy.logerr("%s" % e)





def main():
    try:
           
        rospy.init_node("test_move", anonymous=False)
        test_convert()
        rospy.spin()
        print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
if __name__ == '__main__': main()
