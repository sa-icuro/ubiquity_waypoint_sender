#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: David Butterworth <dbworth@cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
This is a demo of Rviz Tools for python which tests all of the
available functions by publishing lots of Markers in Rviz.
"""

# Python includes
import sys, random, re, tty, os, time
import re

# ROS includes
import roslib
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import rviz_tools_py as rviz_tools


#File includes
import json
import yaml
import shutil
from os.path import isfile, join

filename_pub = rospy.Publisher('route_file', String, queue_size = 1)
setup_pub = rospy.Publisher('setup_response', String, queue_size = 1)
markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')

##### FUNCTIONS TO LOAD FROM FILE #####

def byteify(input):
	if isinstance(input, dict):
		return {byteify(key): byteify(value)
				for key, value in input.iteritems()}
	elif isinstance(input, list):
		return [byteify(element) for element in input]
	elif isinstance(input, unicode):
		return input.encode('utf-8')
	else:
		return input

def isclose(a, b, rel_tol=1e-05, abs_tol=0.0):
	return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

##### END OF FUNCTIONS TO LOAD FROM FILE #####

#For Verbose Funionality set to True
VERBOSE = True

#Data Structures
list_points = [] #(Node_ID, Position_X, Position_Y, Angle?/Quat_Z? )
dict_paths = {}  # Parent_Node_ID: [Connected_Node_IDs]
list_goals = []  #(Position_X, Position_Y, Quat_Z, Wait_Time)

route_path = []  #(Position_X, Position_Y, 0 ) to visualize route paths (WHITE LINE)

#Global Variables/Setup for Callback_setup
request_type = ''

#Global Variables for Callback_safepoint (to create entries in dict_paths{})
node_count = 0

#Global Variables for Callback_path
'''
Only need to remember the parent node's ID in the case we're setting the child, then both reset
#If equal to -1, then a parent node ID has not been set yet (1st safepath-pair member)
If equal to value >= 0, then a parent node ID has been set, and a child is now being connected (2nd safepath-pair member)
'''
parent_node_ID = -1 
p_pose = 0
c_pose = 0

#Global Variables/Setup for Callback_route 
last_route_id = -1
#path_route = []

first_goal_var = True
points_file = '/home/ubuntu/catkin_ws/src/param/session_current/points_current.txt'
paths_file = '/home/ubuntu/catkin_ws/src/param/session_current/paths_current.json'	
route_file = '/home/ubuntu/catkin_ws/src/param/session_current/routes/goals_current.yaml'

def callback_setup(data):
	print "Inside Callback setup"
	if (str(data.data) == "save_session"):
		#Write DS to files and re-save hardcopy DS (curr)
		save_data()
	else:
		globals()['list_points'][:] = []
		globals()['dict_paths'].clear()
		globals()['list_goals'][:] = []
		markers.deleteAllMarkers()
		
		if (str(data.data) == "files"):
			print "-> FILES message recieved!!"
			#Get list of file names and send them
			files = [f for f in os.listdir('/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes')]
			filename_pub.publish(str(files))
			rospy.sleep(0.2)
		elif (str(data.data) == "new_session"):
			print "-> NEW_SESSION message recieved!!"
			#Make new session (save files), make all empty files
			#manage_filesystem(0)
		elif (str(data.data) == "safepoints"):
			print "-> SAFEPOINTS message recieved!!"
			#Make new session (save files), import safepoints ONLY, make empty safepaths and route file
			#manage_filesystem(1)
			#load points (Build DS and visualize)
			#load_points()			
		elif (str(data.data) == "safepaths"):
			print "-> SAFEPATHS message recieved!!"
			#Make new session (save files), import safepoints and safepaths ONLY, make empty route file
			#manage_filesystem(2)
			#load points and paths (Build DS and visualize)
			#load_points()
			#load_paths()
		elif (str(data.data)[-5:] == ".yaml"):
			print "-> File: " + str(data.data) + " recieved!!!"
			#load points and paths (Build DS and visualize)
			#load_points()
			#load_paths()
			#load_goals(str(data.data))
		else: 
			print "-> -> Unknown Input: " + str(data.data)
			pass


def listener():
	rospy.init_node('ubiquity_rviz_server', anonymous=True)
	#Listens to Setup.UI on user's end
	rospy.Subscriber("/setup_requests", String, callback_setup)
	rospy.spin()

if __name__ == '__main__':
	listener()
