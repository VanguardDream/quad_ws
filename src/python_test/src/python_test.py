#!/home/bong/quad_ws/bin/bin/python

import rospy
import sys

rospy.init_node('python_test')
rospy.loginfo("hello world! with %s",sys.version)