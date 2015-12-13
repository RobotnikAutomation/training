#! /usr/bin/env python

import roslib; 
import rospy
import actionlib
import time
import sys

from actionlib_msgs.msg import *
from trajectory_msgs.msg import *

from control_msgs.msg import *

if __name__ == '__main__':
	rospy.init_node('follow_trajectory_test')
	
	client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	
	client.wait_for_server()
	
	if len(sys.argv) < 2:
		option = 1
		print 'Selected option by default %d'%option
	else:
		option = int(sys.argv[1])
		print 'Selected option %d'%option
	
	goal = FollowJointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time()
		
	goal.trajectory.joint_names = ['elbow_joint',
                       'shoulder_lift_joint',
                       'shoulder_pan_joint',
                       'wrist_1_joint',
                       'wrist_2_joint',
                       'wrist_3_joint']
	
	tpoint1 = JointTrajectoryPoint()
	tpoint1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	tpoint1.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint1.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint1.time_from_start = rospy.Duration.from_sec(0.0)
	
	tpoint2 = JointTrajectoryPoint()
	tpoint2.positions = [0.0, -0.7854, 0.0, 0.05, 0.05, 0.05]
	tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint2.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint2.time_from_start = rospy.Duration.from_sec(2.5)
	
	tpoint3 = JointTrajectoryPoint()
	tpoint3.positions = [0.0, -1.5708, 0.0, 0.1, 0.1, 0.1]
	tpoint3.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint3.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
	tpoint3.time_from_start = rospy.Duration.from_sec(5.0)

	
	goal.trajectory.points = [tpoint1, tpoint2, tpoint3]
	
	# Sends 3 trajs
	if option == 1:
		print 'EXECUTING OPTION %d'%option
		
		rospy.loginfo('sending trajectory 1')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
					
		print 'Result is %s'%client.get_result()
		time.sleep(2.0)
		
		tpoint2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		tpoint1.positions = [0.5, -1.0, 0.5, 0.5, 0.0, 0.0]
		tpoint2.positions = [0.6, -1.1, 0.6, 0.6, 0.0, 0.0]
		rospy.loginfo('sending trajectory 2')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		print 'Result is %s'%client.get_result()		
		time.sleep(2.0)
		
		tpoint1.positions = [0.6, -1.1, 0.6, 0.6, 0.0, 0.0]
		tpoint2.positions = [0.5, -1.0, 0.5, 0.5, 0.0, 0.0]	
		rospy.loginfo('sending trajectory 3')
		# Fill in the goal here
		client.send_goal(goal)
		rospy.loginfo('waiting for result')
		while not client.wait_for_result(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown():
			rospy.loginfo('waiting for result. state = %s'%client.get_state())
		print 'Result is %s'%client.get_result()

	# Closing the HAND
	elif option == 2:
		print 'EXECUTING OPTION %d'%option
		
	# Opening the HAND
	elif option == 3:
		print 'EXECUTING OPTION %d'%option
		
