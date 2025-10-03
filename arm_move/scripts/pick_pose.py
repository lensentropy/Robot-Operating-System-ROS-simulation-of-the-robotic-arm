#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import tf

 
moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('reset_pose', anonymous=True)
# robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("manipulator")
joint_state_positions = arm_group.get_current_joint_values()
print (str(joint_state_positions))

# arm_group.set_joint_value_target([ 2.252058185401321, -0.9359317003001024, 1.4311743226467062, 0.49552818900751205, 2.252288922021785, -0.00010918865103892728])
# arm_group.go(wait=True)

cp = arm_group.get_current_pose()
print(cp)

cp.pose.position.x = 0.20
cp.pose.position.y = 0.0
cp.pose.position.z = 0.30

cp.pose.orientation.x = 0.0
cp.pose.orientation.y = 0.707
cp.pose.orientation.z = 0.0
cp.pose.orientation.w = 0.707

arm_group.set_pose_target(cp.pose)
# arm_group.go()

moveit_commander.roscpp_initializer.roscpp_shutdown()
