#!/usr/bin/env python
#coding=utf-8
from __future__ import print_function
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import time
from geometry_msgs.msg import PointStamped,PoseStamped
from move_base_msgs.msg import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
def status_callback(msg):
    global status
    if(msg.status.status == 4):
        status = 4
def txt_processing(path):
    global markerArray,count
    pos = []
    with open(path, 'r') as file_to_read:
        while True:
            lines = file_to_read.readline() 
            if not lines:
                break
            p_tmp = [float(i) for i in lines.split()] 
            pos.append(p_tmp) 
        #------------------------------------------------------------------
            q = tf.transformations.quaternion_from_euler(0, 0, math.radians(p_tmp[2]))
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
        # marker.type = marker.TEXT_VIEW_FACING
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker.pose.position.x = p_tmp[0]
            marker.pose.position.y = p_tmp[1]
            marker.pose.position.z = 0
            markerArray.markers.append(marker)
            count+=1
        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

def computeDistance(point1, point2):
    return math.sqrt((point1.x-point2.x)**2 + (point1.y-point2.y)**2)

def callback_active():
    rospy.loginfo("Action server is processing the %dth goal" % (index))

def callback_feedback(feedback):
    global current_point
    current_point.target_pose.pose.position = feedback.base_position.pose.position
    #rospy.loginfo("Feedback:%s" % str(feedback))

markerArray = MarkerArray()
rospy.init_node('multi_goals_from_txt')

count = 0       #total goal num
index = 0       #current goal point index
status = 3
path = 'path.txt'
txt_processing(path)
rospy.loginfo("navigating to the %dth goal" % (index))
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

current_point = MoveBaseGoal()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position = markerArray.markers[index].pose.position
goal.target_pose.pose.orientation = markerArray.markers[index].pose.orientation
client.send_goal(goal,active_cb=callback_active,feedback_cb=callback_feedback)
rospy.loginfo("navigating to the %dth goal" % (index))

goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
goal_status_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,status_callback)

while(not rospy.is_shutdown()):
	distance = computeDistance(current_point.target_pose.pose.position, goal.target_pose.pose.position)
	if(distance < 0.2):
		rospy.loginfo("achieve the %dth goal successfully" % (index))
		index += 1
		if(index == count):
			rospy.loginfo("all goals achieved!")
			break
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position = markerArray.markers[index].pose.position
		goal.target_pose.pose.orientation = markerArray.markers[index].pose.orientation
		client.send_goal(goal,active_cb=callback_active,feedback_cb=callback_feedback)
		rospy.loginfo("navigating to the %dth goal" % (index))
	if(status == 4):
		rospy.loginfo("failing to the %dth goal" % (index))
		index += 1
		if(index == count):
			rospy.loginfo("all goals achieved!")
			break
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position = markerArray.markers[index].pose.position
		goal.target_pose.pose.orientation = markerArray.markers[index].pose.orientation
		client.send_goal(goal,active_cb=callback_active,feedback_cb=callback_feedback)
		rospy.loginfo("navigating to the %dth goal" % (index))
		status = 3

rospy.spin()
