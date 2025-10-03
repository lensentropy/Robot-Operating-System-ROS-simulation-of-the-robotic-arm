#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf
import time

time.sleep(5)
 
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('reset_pose', anonymous=True)
# robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
joint_state_positions = arm_group.get_current_joint_values()
print (str(joint_state_positions))

arm_group.set_named_target('home_j')
arm_group.go()

moveit_commander.roscpp_initializer.roscpp_shutdown()
