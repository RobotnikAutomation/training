#!/usr/bin/env python

"""
    moveit_cartesian_demo.py - Version 0.1 2014-01-14
    
    Plan and execute a Cartesian path for the end-effector through a number of waypoints
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        #cartesian = rospy.get_param('~cartesian', True)
        cartesian = True
                        
        # Connect to the arm move group
        arm = MoveGroupCommander('manipulator')
        
        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)
        
        # Set the right arm reference frame
        arm.set_pose_reference_frame('base_footprint')
                
        # Allow some leeway in position(meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.02)
        arm.set_goal_orientation_tolerance(0.1)
        
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
                                        
        # Start in the "zero" configuration stored in the SRDF file        
        arm.set_named_target('home')
        
        # Plan and execute a trajectory to the goal configuration
        rospy.loginfo('sending to home')
        arm.go()
        
        # Get the current pose so we can add it as a waypoint
        start_pose = arm.get_current_pose(end_effector_link).pose
                
        # Initialize the waypoints list
        waypoints = []
                
        # Set the first waypoint to be the starting pose
        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(start_pose)
            
        wpose = deepcopy(start_pose)
                
        # Set the next waypoint 
        wpose.position.x -= 0.1 
        wpose.position.y += 0.0
        wpose.position.z += 0.0

        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
         
        # Set the next waypoint
        wpose.position.x -= 0.1
        wpose.position.y += 0.0
        wpose.position.z += 0.0
          
        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)
            
        if cartesian:
            # Append the pose to the waypoints list
            waypoints.append(deepcopy(start_pose))
        else:
            arm.set_pose_target(start_pose)
            arm.go()
            rospy.sleep(1)
            
        if cartesian:
            fraction = 0.0
            maxtries = 100
            attempts = 0
            
            # Set the internal state to the current state
            arm.set_start_state_to_current_state()
     
            # Plan the Cartesian path connecting the waypoints
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses
                                        0.01,        # eef_step 0.01
                                        0.0,        # jump_threshold 0.0
                                        True)        # avoid_collisions
                
                # Increment the number of attempts 
                attempts += 1
                
                # Print out a progress message
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # If we have a complete plan, execute the trajectory
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
    
                arm.execute(plan)
                            
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # Move normally back to the 'resting' position
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
