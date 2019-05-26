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
import sys, tty, os, time, subprocess
import random, re

# ROS includes
import roslib, rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import rviz_tools_py as rviz_tools


#File includes
import json
import yaml
import shutil
from os.path import isfile, join

############### GLOBALS ################
#Declare Publishers
filename_pub = rospy.Publisher('route_file', String, queue_size = 1)
setup_pub = rospy.Publisher('setup_response', String, queue_size = 1)
markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')

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
current_node_ID = 0 
p_pose = 0
c_pose = 0

#Global Variables/Setup for Callback_route 
last_route_id = -1
current_route_node_ID = 0 
#path_route = []

first_goal_var = True
points_file = '/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/points_current.txt'
paths_file = '/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/paths_current.json'	
route_file = '/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes/goals_current.yaml'
########################################

###### FUNCTIONS TO LOAD FROM FILE ######
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
#########################################

############# Loading Tools #############
def reset():
	globals()['list_points'][:] = []
	globals()['node_count'] = 0
	globals()['dict_paths'].clear()
	globals()['parent_node_ID'] = -1
	globals()['child_node_ID'] = 0
	globals()['list_goals'][:] = []
	globals()['route_path'][:] = []
	globals()['current_route_node_ID'] = 0
	markers.deleteAllMarkers()

#For setup options: New_Session, Load_SafePOINTS_Only, and Load_SafePATHS_only
def manage_filesystem(flag):
	if(flag == 0):
		reset()
		#Create New_Session
		#Create new directory and move files from session_current to new_session
		for x in range(1,100):
			dest = "/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_" + str(x)
			if(not os.path.exists(dest)):
				print "Creating Session # " + str(x)
				break

		os.makedirs(dest)
		source = '/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current'
		files = os.listdir(source)
		for f in files:
			shutil.move(source+'/'+f,dest)	
		#create empty points file, paths file, and /routes directory
		open(globals()['points_file'], 'w+')
		open(globals()['paths_file'], "w+")
		os.makedirs(source+'/routes')
	if(flag == 1):
		reset()
		#Safepoints only
		#Create new directory and move files from session_current to new_session
		for x in range(1,100):
			dest = "/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_" + str(x)
			if(not os.path.exists(dest)):
				print "Creating Session # " + str(x)
				break
		
		os.makedirs(dest)
		source = '/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current'
		files = os.listdir(source)
		for f in files:
			shutil.move(source+'/'+f,dest)
		#import safepoints from last /session_current
		pt_src = dest + '/points_current.txt'
		shutil.copyfile(pt_src, (source+'/points_current.txt'))
		#create empty paths file and /route directory
		open(globals()['paths_file'], "w+")
		os.makedirs(source+'/routes')
	if (flag == 2):
		reset()
		#Safepaths only
		#Rename goals_current as goals_x.yaml, and allow user to build new route
		for x in range(1,100):
			dest = "/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes/goals_" + str(x)+".yaml"
			if(not os.path.exists(dest)):
				print "Creating Goals # " + str(x)
				break
		try:
			shutil.move(globals()['route_file'], dest)
		except OSError:
			shutil.move(globals()['route_file'], "/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes/goals_" + str(x+1)+".yaml")
		except:
			pass
		content = open(globals()['route_file'],"w+")
		content.write("goals:\n")

def setup_points():
	globals()['list_points'][:] = []
	with open(globals()['points_file'], 'r') as f:
		for x in f.readlines():
			globals()['node_count'] = len(globals()['list_points'])
			temp_tup1 = x.replace("(","")
			temp_tup2 = temp_tup1.replace(" ","")
			temp_tup1 = temp_tup2.replace(")","")
			temp_tup2 = temp_tup1.split(",")
			temp_tup2[0] = temp_tup2[0].strip("-")
			temp_tup2[3] = temp_tup2[3].replace("\n","")
			globals()['list_points'].append((int(temp_tup2[0]), float(temp_tup2[1]), float(temp_tup2[2]), float(temp_tup2[3])))
			globals()['dict_paths'][str(globals()['node_count'])] = []
		globals()['node_count'] = len(globals()['list_points'])	
	if VERBOSE:
		print "\nVISUALIZING POINTS..."
		print "len safePOINTS: " + str(globals()['node_count'])
		print globals()['list_points']
	do_once = False
	#POPULATE
	for item in globals()['list_points']:
		if (not do_once):
			rospy.sleep(0.2)
			do_once = True

		P = Pose(Point(item[1],item[2],0),Quaternion(0,0,item[3],1))
		rospy.sleep(0.02)
		markers.publishCube(P, 'orange', 0.2, 0) # pose, color, scale, lifetime
		rospy.sleep(0.02)

	if globals()['VERBOSE']:
		print "...DONE VISUALIZING POINTS"

def setup_paths():
	#globals()['dict_paths'].clear()
	globals()['dict_paths'] = dict((k,[]) for k in range(globals()['node_count']))
	try:
		with open(globals()['paths_file'], "r") as f:
			globals()['dict_paths'] = byteify(json.load(f))
	except:
		print "No safepaths in session_current/paths_current.json, or file does not exist!"
		f = open(globals()['paths_file'], "w+")

	if globals()['VERBOSE']:
		print "\nVISUALIZING PATHS:"
		print "len safePATHS: " + str(len(globals()['dict_paths']))
		print globals()['dict_paths']

	#POPULATE
	for key, value in globals()['dict_paths'].iteritems():
		#print globals()['list_points']
		#print "key: " + str(key)
		if(len(value) > 0):
			#print "key (inside method): " + str(key)
			p1 = Point(globals()['list_points'][int(key)][1],globals()['list_points'][int(key)][2],0)
			markers.publishSphere(Pose(p1,Quaternion(0,0,globals()['list_points'][int(key)][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime
			for connected_node_id in value:
				p2 = Point(globals()['list_points'][connected_node_id][1],globals()['list_points'][connected_node_id][2],0)
				markers.publishLine(p1, p2, 'green', 0.05, 0)
				rospy.sleep(0.05)
				markers.publishSphere(Pose(p2,Quaternion(0,0,globals()['list_points'][connected_node_id][3],1)), 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime

	globals()['last_route_ID'] = -1

	if globals()['VERBOSE']:
		print "...DONE VISUALIZING PATHS"

def setup_goals(data):
	globals()['list_goals'][:] = []
	globals()['last_route_ID'] = -1
	globals()['route_path'][:] = []
	globals()['current_route_node_ID'] = 0

	content = ""
	globals()['route_file'] = '/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes/' + str(data)
	#if an exception is thrown here, create file and load empty route.
	try:	
		with open(globals()['route_file'], 'r') as content:
			yml = yaml.load(content)
	except:
		content = open(globals()['route_file'],"w+")
		content.write("goals:\n")
		yml = yaml.load(content)

	try:
		for x in range(0,len(yml['goals'])):
			val = yml['goals'][x].split(',')
			try:
				globals()['list_goals'].append((float(val[0]), float(val[1]), float(val[2]), float(val[3])))
			except:
				#Here 0.0 is the default wait time at each goal/safepoint
				globals()['list_goals'].append((float(val[0]), float(val[1]), float(val[2]), 0.0))
	except:
		print "No route data in session_current/routes/" + str(data) + "!"

	
	if globals()['VERBOSE']:
		print "\nVISUALIZING ROUTE..."
		print "len GOALS: " + str(len(globals()['list_goals']))
		print globals()['list_goals']

	#POPULATE	
	if(len(globals()['list_goals']) > 0):
		for item in globals()['list_goals'][:-1]:
			P = Pose(Point(item[0],item[1],0),Quaternion(0,0,item[2],1))
			scale = Vector3(0.33,0.33,0.33) # diameter
			rospy.sleep(0.02)
			#markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
			markers.publishSphere(P, 'blue', scale, 0) # pose, color, scale, lifetime
			rospy.sleep(0.01)
			globals()['route_path'].append(Point(item[0],item[1],0))
			markers.publishPath(globals()['route_path'], 'white', 0.1, 0)
			rospy.sleep(0.02)
			
		P = Pose(Point(globals()['list_goals'][-1][0],globals()['list_goals'][-1][1],0),Quaternion(0,0,globals()['list_goals'][-1][2],1))
		scale = Vector3(0.33,0.33,0.33) # diameter
		rospy.sleep(0.02)
		markers.publishSphere(P, 'yellow', scale, 0) # pose, color, scale, lifetime
		rospy.sleep(0.01)
		globals()['route_path'].append(Point(globals()['list_goals'][-1][0],globals()['list_goals'][-1][1],0))
		markers.publishPath(globals()['route_path'], 'white', 0.1, 0)
		rospy.sleep(0.02)

	if globals()['VERBOSE']:
		print "...DONE VISUALIZING ROUTE"

def save_data():
	
	print "Starting save process"
	#Save all data (override because sessions have been properly configured)
	f = open(globals()['points_file'],"w+")
	for item in globals()['list_points']:
		f.write("- %d, %f, %f, %f\n" % (item[0], item[1], item[2],item[3]))
	f.close()

	with open(globals()['paths_file'], 'w+') as f:
		json.dump(globals()['dict_paths'], f)

	f = open(globals()['route_file'],"w+")
	f.write("goals:\n")
	for item in globals()['list_goals']:
		f.write("- %f, %f, %f, %f\n" % (item[0], item[1], item[2], item[3]))
	f.close()
	print "Done saving files"
#########################################

########### Processing Tools ############
def callback_safepoint(msg):
	# Copying for simplicity
	position = msg.pose.position
	quat = msg.pose.orientation

	# Publish a sphere using a ROS Pose
	P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
	cube_width = 0.2 # diameter
	color = 'orange'
	markers.publishCube(P, color, cube_width, 0) # pose, color, scale, lifetime

	tup = (globals()['node_count'],position.x,position.y,quat.z)
	globals()['list_points'].append(tup)
	globals()['dict_paths'][str(globals()['node_count'])] = []
	globals()['node_count'] = len(globals()['list_points'])
	if globals()['VERBOSE']:
		print "\n~~~~~~~~ Safepoint Tool ~~~~~~~~"
		print "Appending safepoint: " +  str(tup) + "!"
		print "node_count = " + str(globals()['node_count'])
		#print list_points
		print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
	
def callback_safepath(msg):
	items = str(msg.data).split(" ")
	x, y, z  = float(items[0]), float(items[1]), float(0)
	e_flag = 0

	#Locate Node_ID given x and Y point
	for point in globals()['list_points']:
		if (isclose(point[1],x) and isclose(point[2],y)):
			globals()['current_node_ID'] = int(point[0])
			z = point[3]
			if globals()['VERBOSE']:
				print "\n~~~~~~~~~ Safepath Tool ~~~~~~~~~~"
				print "MATCH!!! with Node_ID# " + str(point[0])
			break
	
	if globals()['VERBOSE']:
		print "Parent_node_ID: " + str(globals()['parent_node_ID'])
		print "Current_node_ID: " + str(globals()['current_node_ID'])
		if (globals()['parent_node_ID'] == -1):
			print "Setting Parent node."
		else:
			print "Connecting Child node."
		print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"


	if (globals()['parent_node_ID'] == -1):
		#Meaning parent node has not been set yet...

		#Create temporary parent node w/ ROS Pose
		globals()['p_pose'] = Pose(Point(x,y,0),Quaternion(0,0,z,1))
		markers.publishSphere(globals()['p_pose'], 'grey', Vector3(0.25,0.25,0.25), 0) # pose, color, scale, lifetime	

		globals()['parent_node_ID'] = globals()['current_node_ID']
		if globals()['VERBOSE']:
			print "Setting Parent (1st value of safepath-pair) node: " + str(globals()['current_node_ID']) + "\n"
			print globals()['dict_paths']
	elif(globals()['parent_node_ID'] >= 0):
		#Meaning Parent node has been set, but child node has not...
		
		#Save child node pose to print later
		globals()['c_pose'] = Pose(Point(x,y,0),Quaternion(0,0,z,1))
		print globals()['current_node_ID']
		#If current node (child) is not a listed connection for the parent node, then append.
		#Exception catches issues with data being parsed in as strings and not converted properly, just in case. 
		if(not((globals()['current_node_ID']) in globals()['dict_paths'][str(globals()['parent_node_ID'])])):
			globals()['dict_paths'][str(globals()['parent_node_ID'])].append((globals()['current_node_ID']))


		#If parent node is not a listed connection for the current node (child), then append.
		#Exception catches issues with data being parsed in as strings and not converted properly, just in case.
		if(not((globals()['parent_node_ID']) in globals()['dict_paths'][str(globals()['current_node_ID'])])):		
			globals()['dict_paths'][str(globals()['current_node_ID'])].append((globals()['parent_node_ID']))


		if globals()['VERBOSE']:
			print "Connecting Child node (= " + str(globals()['current_node_ID']) + ") to parent Node: " + str(globals()['parent_node_ID']) + "\n"
			#print "Parent is an 'int': " + str(type(globals()['parent_node_ID']) is int)
			#print "Child is a 'int': " + str(type(current_node_ID) is int) + "\n"

		#Create and connect Final Parent and Child nodes
		markers.publishSphere(globals()['p_pose'], 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime	
		markers.publishSphere(globals()['c_pose'], 'purple', Vector3(0.275,0.275,0.275), 0) # pose, color, scale, lifetime	
		parent = Point(globals()['list_points'][globals()['parent_node_ID']][1],globals()['list_points'][globals()['parent_node_ID']][2],0)
		child = Point(globals()['list_points'][globals()['current_node_ID']][1],globals()['list_points'][globals()['current_node_ID']][2],0)
		rospy.sleep(0.05)
		markers.publishLine(parent, child, 'green', 0.05, 0)
		rospy.sleep(0.05)

		if globals()['VERBOSE']:
			print "Safepath-Pair successfully created!!!"
			print "Safepath-pair: " + str(globals()['parent_node_ID']) + "-" + str(globals()['current_node_ID']) + " (Parent-Child) appended to safepath dict."
			print "~~~~~~ End of Safepath Tool ~~~~~~\n"
			print globals()['dict_paths']

		#Reset parent variable to False, to indicate a new safepath-pair can be created now
		globals()['parent_node_ID'] = -1

def callback_route(msg):
	items = str(msg.data).split(" ")
	x = float(items[0])
	y = float(items[1])
	z = float(0)

	if(len(globals()['list_goals']) == 0):
		globals()['last_route_id'] = -1
	globals()['current_route_node_ID'] = 0

	#Locate Node_ID given x and Y point
	for point in globals()['list_points']:
		if (isclose(point[1],x) and isclose(point[2],y)):
			globals()['current_route_node_ID'] = point[0]
			z = point[3]
			if globals()['VERBOSE']:
				print "\n~~~~~~~~~~ Route Tool ~~~~~~~~~~"
				print "MATCH!!! with Node_ID# " + str(point[0])
				#print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
			break

	P = Pose(Point(x,y,0),Quaternion(0,0,z,1))
	scale = Vector3(0.33,0.33,0.33) # diameter
	color = 'blue'

	if (globals()['last_route_id'] == -1):
		#Meaning this is the first route goal being set:
		#tup(goal_position_x, goal_position_y, goal_quat_z, goal_wait_time)
		#goal_wait_time default set to '0'
		globals()['list_goals'].append((x,y,z,float(0)))
		markers.publishSphere(P, 'yellow', scale, 0) # pose, color, scale, lifetime
		rospy.sleep(0.01)
		globals()['route_path'].append( Point(x,y,0) )
		globals()['last_route_id'] = globals()['current_route_node_ID']
		if globals()['VERBOSE']:
			print "First route goal (Node #" + str(globals()['current_route_node_ID'])+ ") appended!"
			print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
	else:
		if(globals()['current_route_node_ID'] in globals()['dict_paths'][str(globals()['last_route_id'])]):
			#Check if current node has a safepath from last_node and can be appended to route
				markers.publishSphere(Pose(Point(globals()['list_points'][globals()['last_route_id']][1],globals()['list_points'][globals()['last_route_id']][2],0),Quaternion(0,0,globals()['list_points'][globals()['last_route_id']][3],1)), 'blue', Vector3(0.35,0.35,0.35), 0) # pose, color, scale, lifetime
				markers.publishSphere(P, 'yellow', scale, 0) # pose, color, scale, lifetime
				rospy.sleep(0.01)
				globals()['route_path'].append( Point(x,y,0) )
				markers.publishPath(globals()['route_path'], 'white', 0.1, 0) # path, color, width, lifetime
				if globals()['VERBOSE']:
					print "\nFound current_node (" + str(globals()['current_route_node_ID']) + ") in Node " + str(globals()['last_route_id']) + "'s list of safepaths"
					print "Successfully appending current node to route!"
					print "~~~~~~~ End of Route Tool ~~~~~~\n"
				globals()['last_route_id'] = globals()['current_route_node_ID']
				globals()['list_goals'].append((x,y,z,float(0)))
		else:
			if globals()['VERBOSE']:
				print "~~> Exception! NO safePATH between (" + str(globals()['current_route_node_ID']) + ") and (" + str(globals()['last_route_id']) + ")!"
			pass 

def callback_goalSender(data):
	if (str(data.data) == "RUN"):
		print "Launch goalSender command recieved!: ",str(data.data)
		print subprocess.check_output(["python","goal_sender.py &"])
	elif(str(data.data) == "STOP"):
		print "Kill goalSender command recieved!: ",str(data.data)
		print subprocess.check_output(["pkill -9 -f","goal_sender.py"])
	return

def callback_setup(data):
	print "Inside Callback setup"
	if (str(data.data) == "files"):
		print "-> FILES message recieved!!"
		#Get list of file names and send them
		files = [f for f in os.listdir('/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes')]
		filename_pub.publish(str(files))
		rospy.sleep(0.2)
	elif (str(data.data) == "save_session"):
		#Write DS to files and re-save hardcopy DS (curr)
		save_data()
		print "Done with save function"
	elif (str(data.data) == "new_session"):
		print "-> NEW_SESSION message recieved!!"
		#Make new session (save files), make all empty files
		manage_filesystem(0)
	elif (str(data.data) == "safepoints"):
		print "-> SAFEPOINTS message recieved!!"
		#Make new session (save files), import safepoints ONLY, make empty safepaths and route file
		manage_filesystem(1)
		#Reset DSs, load points (Build DS and visualize)
		setup_points()		
	elif (str(data.data) == "safepaths"):
		print "-> SAFEPATHS message recieved!!"
		#Make new session (save files), import safepoints and safepaths ONLY, make empty route file
		manage_filesystem(2)
		#Reset DSs,load points and paths (Build DS and visualize)
		setup_points()
		setup_paths()
	elif (str(data.data)[-5:] == ".yaml"):
		print "-> File: " + str(data.data) + " recieved!!!"
		#Reset DSs, load points and paths (Build DS and visualize)
		reset()
		setup_points()
		setup_paths()
		setup_goals(str(data.data))
	else: 
		print "-> -> Unknown Input: " + str(data.data)
		pass
	print "Exiting Callback_setup"

#########################################

def listener():

	rospy.init_node('ubiquity_rviz_server', anonymous=True)
	#Listens to Setup.UI on user's end
	rospy.Subscriber("/setup_requests", String, callback_setup)
	rospy.Subscriber("/safepoint_data", PoseStamped, callback_safepoint)
	rospy.Subscriber("/safepath_data", String, callback_safepath)
	rospy.Subscriber("/route_data", String, callback_route)
	rospy.Subscriber("/goal_sender", String, callback_goalSender)
	rospy.spin()

if __name__ == '__main__':
	listener()
