#!/usr/bin/env python
# CODE ADAPTED FROM: https://forum.ubiquityrobotics.com/t/move-basic-tutorial-is-my-map-ok/108/72

import rospy
import yaml

#from math import sin, cos, pi
from math import radians # https://docs.python.org/2/library/math.html
from tf import transformations # for euler to

import time

#ubuntu@magni02:~$ rostopic info /move_base_simple/goal
#Type: geometry_msgs/PoseStamped
from geometry_msgs.msg import PoseStamped

#ubuntu@magni02:~$ rostopic info /move_base/status
#Type: actionlib_msgs/GoalStatusArray
#
#Publishers:
# * /move_basic (http://magni02.local:34847/)
from actionlib_msgs.msg import GoalStatusArray

class GoalSender:

    def __init__(self):
        rospy.init_node("goal_sender")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        #### parameters #######
        self.param_filename = rospy.get_param('~param_filename','/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes/goals_current.yaml')  # where to find the goal list in yaml format
        rospy.loginfo("self.param_filename: %s" % self.param_filename)
        self.rate = rospy.get_param('~rate',20)  # how often do we update in Hz
        rospy.loginfo("self.rate: %s" % str(self.rate))
        self.load_goals()
        self.status = 3 # waiting for a new goal

        #### Publishers ####
        #self.goal_pub = rospy.Publisher("/move_base_simple/goal", geometry_msgs/PoseStamped, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        time.sleep(0.5) # there must be at least  a 0.25sec delay  between publisher creation and message published => for the magni to actually move !?

        #### Subscribers ####
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)



    def load_goals(self):
        content = ""
        with open('/home/ubuntu/catkin_ws/src/ubiquity_tools/param/session_current/routes/goals_current.yaml', 'r') as content_file:
            content = content_file.read()
        #print(yaml.dump(yaml.load(content)))
        yml = yaml.load(content)
        self.goals = yml['goals']
        self.index = 0
        rospy.loginfo("done loading goals")

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        # known statuses
        #status: 1 | text: "This goal has been accepted by the simple action server"]  | magni is executing the goal command
        #status: 3 | text: ""]  | the magni is waiting for a goal

        if self.status == 3:
            goal = self.goals[self.index].split(',')
            # sending goal using command line
            # ubuntu@magni02:~$ rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
            # publishing and latching message for 3.0 seconds
            # ubuntu@magni02:~$
            time.sleep(0.25)
            pose = self.to_pose_stamped(float(goal[0]), float(goal[1]), float(goal[2]))
            rospy.loginfo("Sending goal pose "+str(self.index)+" to /move_base_simple/goal")
            #rospy.loginfo(str(pose))

            self.goal_pub.publish(pose)
            self.index += 1
            if self.index >= len(self.goals):
                self.index = 0
            time.sleep(0.5) # give time for callback to be called

        elif self.status == 1:
            x = 0 # do nothing since magni is executing the move command
            # status_callback will move the current status back to '3' once the goal is reached
            #rospy.loginfo("waiting for goal to be reached")
        else:
            rospy.logwarn("unknown status: %s (Check obstacles!)" % self.status)

    def to_pose_stamped(self,x,y,theta_in_degrees):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        quaternion = transformations.quaternion_from_euler(0.0, 0.0, radians(theta_in_degrees))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose
    # source of inspiration => https://github.com/xaxxontech/oculusprime_ros/blob/master/src/global_path_follower.py
    def status_callback(self, msg):
        #rospy.loginfo(msg)
        #rospy.loginfo(msg.status_list)
        if len(msg.status_list) == 0:
            return

        self.status = msg.status_list[len(msg.status_list)-1].status # get latest status

if __name__ == '__main__':
    """ main """
    goal_sender = GoalSender()
    goal_sender.spin()
