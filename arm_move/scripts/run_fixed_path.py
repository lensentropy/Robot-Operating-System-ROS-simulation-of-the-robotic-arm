#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import  Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import sys
import tf
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from copy import deepcopy

class MoveRobot():
    def __init__(self):
        # 初始化
        self.robot = moveit_commander.RobotCommander()
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.poses_array = []  # 用于存放从文件加载的路点
        self.listener = tf.TransformListener()
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.path_publisher = rospy.Publisher('robot_path', Path, queue_size=10)  # 添加路径发布者

        # 物体吸附接口

        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                             Attach)
        self.attach_srv.wait_for_service()

        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                             Attach)
        self.detach_srv.wait_for_service()

    def gazeboAttach(self):

        rospy.loginfo("Attaching gripper and object")
        req = AttachRequest()

        req.model_name_1 = "robot"
        req.link_name_1 = "wrist_3_link"

        req.model_name_2 = "Shiguan_urdf"
        req.link_name_2 = "base_link"

        self.attach_srv.call(req)

    def gazeboDetach(self):

        rospy.loginfo("Detaching gripper and object")
        req = AttachRequest()

        req.model_name_1 = "robot"
        req.link_name_1 = "wrist_3_link"

        req.model_name_2 = "Shiguan_urdf"
        req.link_name_2 = "base_link"

        self.detach_srv.call(req)

    def load_waypoints_from_file(self, file_path):
        """从文本文件中加载路点"""
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    x, y, z = [float(val) for val in line.strip().split()]
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = z - 0.35
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.707
                    pose.orientation.z = 0.0
                    pose.orientation.w = 0.707
                    self.poses_array.append(pose)
            rospy.loginfo("Waypoints loaded successfully from file.")
        except Exception as e:
            rospy.logerr("Error reading waypoints file: {}".format(e))

    def plan_and_move_to_waypoints(self):
        """计划并移动到从文件中加载的路点，并发布下沉 -0.23 的 Path"""
        # 使用拷贝的方式构造 waypoints，避免对原数组产生副作用
        waypoints = [p for p in self.poses_array]

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,        # waypoints to follow
            0.01,             # eef_step
            0.0,              # jump_threshold
            True              # avoid_collisions
        )

        if fraction == 1.0:
            # 创建并发布 Path 消息（z 统一下沉 0.23）
            path_msg = Path()
            path_msg.header.frame_id = "world"
            path_msg.header.stamp = rospy.Time.now()

            for pose in waypoints:
                ps = PoseStamped()
                ps.header = path_msg.header
                # 这里不要直接赋 = pose，显式创建新的 Pose 并下沉 z
                ps.pose = Pose(
                    position=Point(
                        x=pose.position.x,
                        y=pose.position.y,
                        z=pose.position.z - 0.23
                    ),
                    orientation=Quaternion(
                        x=pose.orientation.x,
                        y=pose.orientation.y,
                        z=pose.orientation.z,
                        w=pose.orientation.w
                    )
                )
                path_msg.poses.append(ps)

            rospy.loginfo("Path planning successful.")

            plan = self.arm_group.retime_trajectory(self.robot.get_current_state(), plan, 0.01)

            # 执行并发布路径
            self.arm_group.execute(plan, wait=True)
            self.path_publisher.publish(path_msg)
        else:
            rospy.logwarn("Path planning failed with only {}".format(fraction))


    def publish_waypoints_as_markers(self):
        rospy.sleep(0.5)
        ma = MarkerArray()
        stamp = rospy.Time.now()

        for i, p in enumerate(self.poses_array):
            base = p.pose if hasattr(p, "pose") else p  # 统一成 Pose

            m = Marker()
            m.header.frame_id = "world"   # 或者 base 对应的坐标系
            m.header.stamp = stamp
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            # 显式拷贝 + 下沉 0.80m
            m.pose = Pose(
                position=Point(
                    x=base.position.x,
                    y=base.position.y,
                    z=base.position.z - 0.23
                ),
                orientation=Quaternion(
                    x=base.orientation.x,
                    y=base.orientation.y,
                    z=base.orientation.z,
                    w=base.orientation.w
                )
            )

            m.scale.x = m.scale.y = m.scale.z = 0.005  # 调大点便于观察
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0

            rospy.logwarn("id=%d  orig z=%.3f  new z=%.3f",
                        i, base.position.z, m.pose.position.z)

            ma.markers.append(m)

        self.marker_publisher.publish(ma)


    def main_loop(self):

        # gripper object

        self.arm_group.set_joint_value_target([0.8876911805441035, -1.6403008707803792, 1.6522049658292035, -1.6069353306570653, -1.582915668207634, 0.8873042922399073])
        self.arm_group.go()

        self.arm_group.set_joint_value_target([0.8858397158160685, -1.6194698580899631, 1.8134706252644843, -1.7888290162281013, -1.5830270950633203, 0.8855142932518021])
        self.arm_group.go()

        self.gripper_group.set_named_target('close')
        self.gripper_group.go()
        
        self.gazeboAttach()

        self.arm_group.set_joint_value_target([0.8876911805441035, -1.6403008707803792, 1.6522049658292035, -1.6069353306570653, -1.582915668207634, 0.8873042922399073])
        self.arm_group.go()

        # run fixed path
        file_path = '/home/zhitao/diskE/Project/homework9_summation/hw34_yaogao_ws/data/path.txt'  # Update this path to your file location
        self.load_waypoints_from_file(file_path)

        self.arm_group.set_named_target('home_j')
        self.arm_group.go()
        
        rospy.sleep(1)  # Wait for the arm to reach the home position
        self.publish_waypoints_as_markers()
        self.plan_and_move_to_waypoints()

        # place object
        self.arm_group.set_named_target('home_j')
        self.arm_group.go()

        self.arm_group.set_joint_value_target([0.8876911805441035, -1.6403008707803792, 1.6522049658292035, -1.6069353306570653, -1.582915668207634, 0.8873042922399073])
        self.arm_group.go()

        self.arm_group.set_joint_value_target([0.8858397158160685, -1.6194698580899631, 1.8134706252644843, -1.7888290162281013, -1.5830270950633203, 0.8855142932518021])
        self.arm_group.go()

        self.gazeboDetach()

        self.arm_group.set_joint_value_target([0.8876911805441035, -1.6403008707803792, 1.6522049658292035, -1.6069353306570653, -1.582915668207634, 0.8873042922399073])
        self.arm_group.go()

        self.arm_group.set_named_target('home_j')
        self.arm_group.go()

def main():
    rospy.init_node('move_robot_demo', anonymous=True)
    rospy.loginfo('Starting Move Robot Demo')
    move_robot = MoveRobot()

    try:
        move_robot.main_loop()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
