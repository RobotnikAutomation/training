#!/usr/bin/env python

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand
import moveit_msgs.msg
import geometry_msgs.msg

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        print "============ FK demo"

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        
        GRIPPER_OPEN = [0.0, 0.0]
        GRIPPER_CLOSED = [0.6, 0.6]
 
        # Connect to the arm move group
        arm = moveit_commander.MoveGroupCommander('arm')
        
        # Connect to the hand move group
        hand = moveit_commander.MoveGroupCommander('hand')
                
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        # rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        arm.set_goal_joint_tolerance(0.001)
        hand.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "resting" pose stored in the SRDF file
        arm.set_named_target('zero')
        
        # Plan a trajectory to the goal configuration
        traj = arm.plan()
         
        # Execute the planned trajectory
        rospy.loginfo("going to zero")
        arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
        
        # Set the gripper target to neutal position using a joint value target        
        hand.set_joint_value_target(GRIPPER_OPEN)         
        
        # Plan and execute the gripper motion
        rospy.loginfo("opening gripper")
        hand.go()
        rospy.sleep(1)
         
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [0.3, -0.3, 0.3, 0.3, 0.3, 0.3]
 
        # Set the arm's goal configuration to the be the joint positions
        arm.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        rospy.loginfo("going to joint_positions")
        arm.go()
        rospy.sleep(1)
         
        # Save this configuration for later
        arm.remember_joint_values('saved_config', joint_positions)
         
        # Close the gripper as if picking something up
        hand.set_joint_value_target(GRIPPER_CLOSED)
        rospy.loginfo("closing gripper")
        hand.go()
        rospy.sleep(1)
                 
        # Set the arm target to the named "pregrasp1" pose stored in the SRDF file
        arm.set_named_target('pregrasp1')
         
        # Plan and execute the motion
        rospy.loginfo("going to pregrasp")
        arm.go()
        rospy.sleep(1)
                  
        # Set the goal configuration to the named configuration saved earlier
        arm.set_named_target('saved_config')
         
        # Plan and execute the motion
        rospy.loginfo("going to joint_positions")
        arm.go()
        rospy.sleep(1)
         
        # Open the gripper as if letting something go
        hand.set_joint_value_target(GRIPPER_OPEN)
        rospy.loginfo("opening gripper")
        hand.go()
        rospy.sleep(1)
         
        # Return the arm to the named "zero" pose stored in the SRDF file
        arm.set_named_target('zero')
        rospy.loginfo("going to zero")
        arm.go()
        rospy.sleep(1)
         
        # Return the gripper target to neutral position
        hand.set_joint_value_target(GRIPPER_OPEN)
        rospy.loginfo("opening gripper")
        hand.go()
        rospy.sleep(1)
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
