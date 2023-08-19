#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf, actionlib
import actionlib_msgs.msg
import sys
from rosplan_planning_system.ActionInterfacePy.RPActionInterface import RPActionInterface
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import copy
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import *
import pickle
import math
import time
import json
import pandas as pd
import json
import os
# import yaml
import numpy as np
from sklearn.svm import LinearSVC
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
import subprocess
import signal
import joblib
import requests
# from ruamel.yaml import YAML


FETCH_DATA_HEADERS_123 = [
    "l_wheel_joint_position",
    "r_wheel_joint_position",
    "torso_lift_joint_position",
    "bellows_joint_position",
    "head_pan_joint_position",
    "head_tilt_joint_position",
    "shoulder_pan_joint_position",
    "shoulder_lift_joint_position",
    "upperarm_roll_joint_position",
    "elbow_flex_joint_position",
    "forearm_roll_joint_position",
    "wrist_flex_joint_position",
    "wrist_roll_joint_position",
    "l_gripper_finger_joint_position",
    "r_gripper_finger_joint_position",
    "l_wheel_joint_velocity",
    "r_wheel_joint_velocity",
    "torso_lift_joint_velocity",
    "bellows_joint_velocity",
    "head_pan_joint_velocity",
    "head_tilt_joint_velocity",
    "shoulder_pan_joint_velocity",
    "shoulder_lift_joint_velocity",
    "upperarm_roll_joint_velocity",
    "elbow_flex_joint_velocity",
    "forearm_roll_joint_velocity",
    "wrist_flex_joint_velocity",
    "wrist_roll_joint_velocity",
    "l_gripper_finger_joint_velocity",
    "r_gripper_finger_joint_velocity",
    "l_wheel_joint_effort",
    "r_wheel_joint_effort",
    "torso_lift_joint_effort",
    "bellows_joint_effort",
    "head_pan_joint_effort",
    "head_tilt_joint_effort",
    "shoulder_pan_joint_effort",
    "shoulder_lift_joint_effort",
    "upperarm_roll_joint_effort",
    "elbow_flex_joint_effort",
    "forearm_roll_joint_effort",
    "wrist_flex_joint_effort",
    "wrist_roll_joint_effort",
    "l_gripper_finger_joint_effort",
    "r_gripper_finger_joint_effort",
    "ground_plane_pose_position_x",
    "ground_plane_pose_position_y",
    "ground_plane_pose_position_z",
    "ground_plane_pose_orientation_x",
    "ground_plane_pose_orientation_y",
    "ground_plane_pose_orientation_z",
    "ground_plane_pose_orientation_w",
    "test_zone_pose_position_x",
    "test_zone_pose_position_y",
    "test_zone_pose_position_z",
    "test_zone_pose_orientation_x",
    "test_zone_pose_orientation_y",
    "test_zone_pose_orientation_z",
    "test_zone_pose_orientation_w",
    "table1_pose_position_x",
    "table1_pose_position_y",
    "table1_pose_position_z",
    "table1_pose_orientation_x",
    "table1_pose_orientation_y",
    "table1_pose_orientation_z",
    "table1_pose_orientation_w",
    "table2_pose_position_x",
    "table2_pose_position_y",
    "table2_pose_position_z",
    "table2_pose_orientation_x",
    "table2_pose_orientation_y",
    "table2_pose_orientation_z",
    "table2_pose_orientation_w",
    "demo_cube_pose_position_x",
    "demo_cube_pose_position_y",
    "demo_cube_pose_position_z",
    "demo_cube_pose_orientation_x",
    "demo_cube_pose_orientation_y",
    "demo_cube_pose_orientation_z",
    "demo_cube_pose_orientation_w",
    "fetch_pose_position_x",
    "fetch_pose_position_y",
    "fetch_pose_position_z",
    "fetch_pose_orientation_x",
    "fetch_pose_orientation_y",
    "fetch_pose_orientation_z",
    "fetch_pose_orientation_w",
    "ground_plane_twist_linear_x",
    "ground_plane_twist_linear_y",
    "ground_plane_twist_linear_z",
    "ground_plane_twist_angular_x",
    "ground_plane_twist_angular_y",
    "ground_plane_twist_angular_z",
    "test_zone_twist_linear_x",
    "test_zone_twist_linear_y",
    "test_zone_twist_linear_z",
    "test_zone_twist_angular_x",
    "test_zone_twist_angular_y",
    "test_zone_twist_angular_z",
    "table1_twist_linear_x",
    "table1_twist_linear_y",
    "table1_twist_linear_z",
    "table1_twist_angular_x",
    "table1_twist_angular_y",
    "table1_twist_angular_z",
    "table2_twist_linear_x",
    "table2_twist_linear_y",
    "table2_twist_linear_z",
    "table2_twist_angular_x",
    "table2_twist_angular_y",
    "table2_twist_angular_z",
    "demo_cube_twist_linear_x",
    "demo_cube_twist_linear_y",
    "demo_cube_twist_linear_z",
    "demo_cube_twist_angular_x",
    "demo_cube_twist_angular_y",
    "demo_cube_twist_angular_z",
    "fetch_twist_linear_x",
    "fetch_twist_linear_y",
    "fetch_twist_linear_z",
    "fetch_twist_angular_x",
    "fetch_twist_angular_y",
    "fetch_twist_angular_z",
]

# fetch joint + demo_cube + fetch object + table1 + table2
# 39 + 52
FETCH_VALID_HEADERS_91 = [
    "l_wheel_joint_position",
    "r_wheel_joint_position",
    "torso_lift_joint_position",
    "bellows_joint_position",
    "shoulder_pan_joint_position",
    "shoulder_lift_joint_position",
    "upperarm_roll_joint_position",
    "elbow_flex_joint_position",
    "forearm_roll_joint_position",
    "wrist_flex_joint_position",
    "wrist_roll_joint_position",
    "l_gripper_finger_joint_position",
    "r_gripper_finger_joint_position",
    "l_wheel_joint_velocity",
    "r_wheel_joint_velocity",
    "torso_lift_joint_velocity",
    "bellows_joint_velocity",
    "shoulder_pan_joint_velocity",
    "shoulder_lift_joint_velocity",
    "upperarm_roll_joint_velocity",
    "elbow_flex_joint_velocity",
    "forearm_roll_joint_velocity",
    "wrist_flex_joint_velocity",
    "wrist_roll_joint_velocity",
    "l_gripper_finger_joint_velocity",
    "r_gripper_finger_joint_velocity",
    "l_wheel_joint_effort",
    "r_wheel_joint_effort",
    "torso_lift_joint_effort",
    "bellows_joint_effort",
    "shoulder_pan_joint_effort",
    "shoulder_lift_joint_effort",
    "upperarm_roll_joint_effort",
    "elbow_flex_joint_effort",
    "forearm_roll_joint_effort",
    "wrist_flex_joint_effort",
    "wrist_roll_joint_effort",
    "l_gripper_finger_joint_effort",
    "r_gripper_finger_joint_effort",
    "table1_pose_position_x",
    "table1_pose_position_y",
    "table1_pose_position_z",
    "table1_pose_orientation_x",
    "table1_pose_orientation_y",
    "table1_pose_orientation_z",
    "table1_pose_orientation_w",
    "table2_pose_position_x",
    "table2_pose_position_y",
    "table2_pose_position_z",
    "table2_pose_orientation_x",
    "table2_pose_orientation_y",
    "table2_pose_orientation_z",
    "table2_pose_orientation_w",
    "demo_cube_pose_position_x",
    "demo_cube_pose_position_y",
    "demo_cube_pose_position_z",
    "demo_cube_pose_orientation_x",
    "demo_cube_pose_orientation_y",
    "demo_cube_pose_orientation_z",
    "demo_cube_pose_orientation_w",
    "fetch_pose_position_x",
    "fetch_pose_position_y",
    "fetch_pose_position_z",
    "fetch_pose_orientation_x",
    "fetch_pose_orientation_y",
    "fetch_pose_orientation_z",
    "fetch_pose_orientation_w",
    "table1_twist_linear_x",
    "table1_twist_linear_y",
    "table1_twist_linear_z",
    "table1_twist_angular_x",
    "table1_twist_angular_y",
    "table1_twist_angular_z",
    "table2_twist_linear_x",
    "table2_twist_linear_y",
    "table2_twist_linear_z",
    "table2_twist_angular_x",
    "table2_twist_angular_y",
    "table2_twist_angular_z",
    "demo_cube_twist_linear_x",
    "demo_cube_twist_linear_y",
    "demo_cube_twist_linear_z",
    "demo_cube_twist_angular_x",
    "demo_cube_twist_angular_y",
    "demo_cube_twist_angular_z",
    "fetch_twist_linear_x",
    "fetch_twist_linear_y",
    "fetch_twist_linear_z",
    "fetch_twist_angular_x",
    "fetch_twist_angular_y",
    "fetch_twist_angular_z",
]

FETCH_VALID_HEADERS_65 = [
    "l_wheel_joint_position",
    "r_wheel_joint_position",
    "torso_lift_joint_position",
    "bellows_joint_position",
    "shoulder_pan_joint_position",
    "shoulder_lift_joint_position",
    "upperarm_roll_joint_position",
    "elbow_flex_joint_position",
    "forearm_roll_joint_position",
    "wrist_flex_joint_position",
    "wrist_roll_joint_position",
    "l_gripper_finger_joint_position",
    "r_gripper_finger_joint_position",
    "l_wheel_joint_velocity",
    "r_wheel_joint_velocity",
    "torso_lift_joint_velocity",
    "bellows_joint_velocity",
    "shoulder_pan_joint_velocity",
    "shoulder_lift_joint_velocity",
    "upperarm_roll_joint_velocity",
    "elbow_flex_joint_velocity",
    "forearm_roll_joint_velocity",
    "wrist_flex_joint_velocity",
    "wrist_roll_joint_velocity",
    "l_gripper_finger_joint_velocity",
    "r_gripper_finger_joint_velocity",
    "l_wheel_joint_effort",
    "r_wheel_joint_effort",
    "torso_lift_joint_effort",
    "bellows_joint_effort",
    "shoulder_pan_joint_effort",
    "shoulder_lift_joint_effort",
    "upperarm_roll_joint_effort",
    "elbow_flex_joint_effort",
    "forearm_roll_joint_effort",
    "wrist_flex_joint_effort",
    "wrist_roll_joint_effort",
    "l_gripper_finger_joint_effort",
    "r_gripper_finger_joint_effort",
    "demo_cube_pose_position_x",
    "demo_cube_pose_position_y",
    "demo_cube_pose_position_z",
    "demo_cube_pose_orientation_x",
    "demo_cube_pose_orientation_y",
    "demo_cube_pose_orientation_z",
    "demo_cube_pose_orientation_w",
    "fetch_pose_position_x",
    "fetch_pose_position_y",
    "fetch_pose_position_z",
    "fetch_pose_orientation_x",
    "fetch_pose_orientation_y",
    "fetch_pose_orientation_z",
    "fetch_pose_orientation_w",
    "demo_cube_twist_linear_x",
    "demo_cube_twist_linear_y",
    "demo_cube_twist_linear_z",
    "demo_cube_twist_angular_x",
    "demo_cube_twist_angular_y",
    "demo_cube_twist_angular_z",
    "fetch_twist_linear_x",
    "fetch_twist_linear_y",
    "fetch_twist_linear_z",
    "fetch_twist_angular_x",
    "fetch_twist_angular_y",
    "fetch_twist_angular_z",
]
# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0])

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0])

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            if obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success,pick_result

    def place(self, block, pose_stamped,pick_result):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

class RPMoveBasePy(RPActionInterface):
    def __init__(self):
        # call parent constructor
        RPActionInterface.__init__(self)
        # setup a move base clear costmap client (to be able to send clear costmap requests later on)
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_srv_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # get waypoints reference frame from param server
        self.waypoint_frameid = rospy.get_param('~waypoint_frameid', 'map')
        self.wp_namespace = rospy.get_param('~wp_namespace', '/rosplan_demo_waypoints/wp')
        # create move base action client
        actionserver = rospy.get_param('~action_server', '/move_base')
        self.action_client = actionlib.SimpleActionClient(actionserver, MoveBaseAction)



    def saveData(self,cube):
        f = open("~/bagfiles/data.txt", 'w')
        pose = PoseStamped()
        pose.pose =   cube.primitive_poses[0]
        f.write(cube.header.frame_id+'\n')
        f.write(cube.name+'\n')
        f.write(str(pose.pose.position.x)+'\n')
        f.write(str(pose.pose.position.y)+'\n')
        f.write(str(pose.pose.position.z)+'\n')
        f.write(str(pose.pose.orientation.x)+'\n')
        f.write(str(pose.pose.orientation.y)+'\n')
        f.write(str(pose.pose.orientation.z)+'\n')
        f.write(str(pose.pose.orientation.w)+'\n')
        print(type(cube.header.frame_id))
        f.close()

    def readData(self):
        f = open("~/bagfiles/data.txt", 'w')
        lines = f.readlines()
        data = []
        for line in lines:
            data.append(line)
        pose = PoseStamped
        pose.pose.position.x = float(data[2])
        pose.pose.position.y = float(data[3])
        pose.pose.position.z = float(data[4])
        pose.pose.orientation.x = float(data[5])
        pose.pose.orientation.y = float(data[6])
        pose.pose.orientation.z = float(data[7])
        pose.pose.orientation.w = float(data[8])
        pose.header.frame_id = data[0]
        return pose,data[1]

    def concreteCallback(self, msg):
        position = 0
        filr_path1='~/bagfiles/baseline/'+str(position)
        if not os.path.exists(filr_path1):
            os.mkdir(filr_path1)

        if not self.paramenter(['table1','table2','demo_cube']):
            rospy.logerr('failed to get pose from gazebo')
        lenth_of_msg = len(msg.parameters)
	print(msg)
        name = msg.name

        #move action
        if(name == 'move'):
            # Setup clients
            move_base = MoveBaseClient()
            rospy.loginfo('move to talbe...')
            model_name = 'demo_cube'
            which = 'move'
            path1=filr_path1+'/joint_state_{}.txt'.format(which) 
            path2=filr_path1+'/object_state_{}.txt'.format(which)
            x1 = rospy.get_param(model_name+'_x')-1.55
            y1 = rospy.get_param(model_name+'_y')-0.032
            x2 = rospy.get_param(model_name+'_x')-1.35
            y2 = rospy.get_param(model_name+'_y')-0.032
	    print(x1,y1)
	    print(x2,y2)
            p1_move1 = subprocess.Popen('rostopic echo /joint_states > '+path1,close_fds=True,preexec_fn = os.setsid,shell = True)
            p2_move1 = subprocess.Popen('rostopic echo /gazebo/model_states > '+path2,close_fds=True,preexec_fn = os.setsid,shell = True)
	    move_base.goto(x1, y1,0.0)
            move_base.goto(x2,y2,0.0)
            
            os.killpg( p1_move1.pid,signal.SIGUSR1)
            os.killpg( p2_move1.pid,signal.SIGUSR1)
            # os.killpg(os.getpgid(p1_move1),9)
            # os.killpg(os.getpgid(p2_move1),9)
            state_check=json.loads(check_symbol(path1, path2,which))
            print(state_check)
            # for num in state_check.values():
            #     if num!=1:
            #         rospy.loginfo('KCL: the final position does not satisfy the target of robot_beside')
            #         return False
            return True
        elif name == 'pick':
            model_name = 'demo_cube'
            which = 'pick_cube'
            path1=filr_path1+'/joint_state_{}.txt'.format(which)
            path2=filr_path1+'/object_state_{}.txt'.format(which)

            # create torso action client
            torso_action = FollowTrajectoryClient("torso_controller",["torso_lift_joint"])
            # creat head action client
            head_action = PointHeadClient()
            # creat grasping client
            grasping_client = GraspingClient()

            #if not self.block_grasp() or self.distant('table1'):
            #    rospy.loginfo('KCL: the orial position can not satisfy the precondition')
            #    return False
            rospy.loginfo("Raising torso...")
	    p1_move1 = subprocess.Popen('rostopic echo /joint_states > '+path1,close_fds=True,preexec_fn = os.setsid,shell = True)
            p2_move1 = subprocess.Popen('rostopic echo /gazebo/model_states > '+path2,close_fds=True,preexec_fn = os.setsid,shell = True)
            # Raise the torso using just a controller
            torso_action.move_to([0.4, ])
            # Point the head at the cube we want to pick
            head_action.look_at(rospy.get_param(model_name+'_x')-0.1,rospy.get_param(model_name+'_y'),rospy.get_param(model_name+'_z'), "map")
            # Get block to pick
            while not rospy.is_shutdown():
                rospy.loginfo("Picking object...")
                grasping_client.updateScene()
                cube, grasps = grasping_client.getGraspableCube()
                if cube == None:
                    rospy.logwarn("Perception failed.")
                    continue
                # Pick the block
                result,pick_result = grasping_client.pick(cube, grasps)
                file1 = open("~/bagfiles/cube.json","wb")
                pickle.dump(cube,file1)
                file1.close
                file2 = open("~/bagfiles/result.json","wb")
                pickle.dump(pick_result,file2)
                file2.close
                if result:
                    break
                rospy.logwarn("Grasping failed.")
                return False
            # Tuck the arm
            grasping_client.tuck()

            # Lower torso
            rospy.loginfo("Lowering torso...")
            torso_action.move_to([0.0, ])
            
            os.killpg( p1_move1.pid,signal.SIGUSR1)
            os.killpg( p2_move1.pid,signal.SIGUSR1)

            # os.killpg(os.getpgid(p1_move1),9)
            # os.killpg(os.getpgid(p2_move1),9)
            state_check=json.loads(check_symbol(path1, path2,which))
            print(state_check)
            for num in state_check.values():
                if num!=1:
                    rospy.loginfo('KCL: the final position can not satisfy the target')
                    return False
            return True

        elif name == 'place':
            which = 'place_cube'
            path1=filr_path1+'/joint_state_{}.txt'.format(which)
            path2=filr_path1+'/object_state_{}.txt'.format(which)
            p1_move1 = subprocess.Popen('rostopic echo /joint_states > '+path1,close_fds=True,preexec_fn = os.setsid,shell = True)
            p2_move1 = subprocess.Popen('rostopic echo /gazebo/model_states > '+path2,close_fds=True,preexec_fn = os.setsid,shell = True)
            # create torso action client
            torso_action = FollowTrajectoryClient("torso_controller",["torso_lift_joint"])
            # creat head action client
            head_action = PointHeadClient()
            # creat grasping client
            grasping_client = GraspingClient()

            #if not self.block_grasp() or self.distant('table2'):
            #    rospy.loginfo('KCL: the orial position can not satisfy the precondition')
            #    return False
            # Raise the torso using just a controller
            rospy.loginfo("Raising torso...")
            torso_action.move_to([0.4, ])
            # Place the block
            file1 = open("~/bagfiles/cube.json","r")
            cube = pickle.load(file1)
            file1.close
            file2 = open("~/bagfiles/result.json","r")
            pick_result = pickle.load(file2)
            file2.close
            while not rospy.is_shutdown():
                        rospy.loginfo("Placing object...")
                        pose = PoseStamped()
                        pose.pose = cube.primitive_poses[0]
                        pose.pose.position.z += 0.05
                        pose.header.frame_id = cube.header.frame_id
                        if grasping_client.place(cube, pose,pick_result):
                            break
                        rospy.logwarn("Placing failed.")
                        return False
            # Tuck the arm, lower the torso
            grasping_client.tuck()

            torso_action.move_to([0.0, ])
            os.killpg( p1_move1.pid,signal.SIGUSR1)
            os.killpg( p2_move1.pid,signal.SIGUSR1)
            # os.killpg(os.getpgid(p1_move1.pid),9)
            # os.killpg(os.getpgid(p2_move1.pid),9)
            state_check=json.loads(check_symbol(path1, path2,which))
            print(state_check)
            for num in state_check.values():
                if num!=1:
                    rospy.loginfo('KCL: the final position can not satisfy the target')
                    return False
            pose_save(filr_path1,{'table1','table2','demo_cube'},'a')
            return True

        
        #transport action
        elif name == 'transport':
            #if not self.block_grasp() or self.distant('table1'):
            #    rospy.loginfo('KCL: the orial position can not satisfy the precondition')
            #    return False
            # Setup clients
            move_base = MoveBaseClient()
            rospy.loginfo('transport to talbe...')
            model_name = 'table2'
            which = 'transport'
            path1=filr_path1+'/joint_state_{}.txt'.format(which)
            path2=filr_path1+'/object_state_{}.txt'.format(which)
            x1 = rospy.get_param(model_name+'_x')-0.3
            y1 = rospy.get_param(model_name+'_y')-1.55
            x2 = rospy.get_param(model_name+'_x')-0.3
            y2 = rospy.get_param(model_name+'_y')-1.05
            p1_move1 = subprocess.Popen('rostopic echo /joint_states > '+path1,close_fds=True,preexec_fn = os.setsid,shell = True)
            p2_move1 = subprocess.Popen('rostopic echo /gazebo/model_states > '+path2,close_fds=True,preexec_fn = os.setsid,shell = True)
	    move_base.goto(x1,y1,1.57)
            move_base.goto(x2,y2,1.57)
            os.killpg( p1_move1.pid,signal.SIGUSR1)
            os.killpg( p2_move1.pid,signal.SIGUSR1)
            # os.killpg(os.getpgid(p1_move1),9)
            # os.killpg(os.getpgid(p2_move1),9)
            state_check=json.loads(check_symbol(path1, path2,which))
            print(state_check)
            for num in state_check.values():
                if num!=1:
                    rospy.loginfo('KCL: the final position can not satisfy the target')
                    return False
            return True
        else:
            rospy.loginfo('the lenth of plan invalid')
            return False
    
    def paramenter(self,models):
        get_state_service = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        model = GetModelStateRequest()
        for model_name in models:
            model.model_name = model_name
            objstate = get_state_service(model)
            rospy.set_param(model_name+'_x',objstate.pose.position.x)
            rospy.set_param(model_name+'_y',objstate.pose.position.y)
            rospy.set_param(model_name+'_z',objstate.pose.position.z)
        return True

def pose_save(path,models,power):
    f = open(path + '/position.txt',power)
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    model = GetModelStateRequest() 
    for i in models:
        model.model_name = i
        objstate = get_state_service(model)
	f.write(i + ':' + '\n')
        f.write(str(objstate.pose.position) + '\n')
    f.close

def read_yaml_data(file_path, file_type):
    """
    @description  :  read fetch data from a yaml joint/state file
    ---------
    @param  :  file_path -> string
               file_type -> joint_state/object_pose
    -------
    @Returns  : list[, []]
    -------
    """

    if os.path.isfile(file_path):
        with open(file_path, "r") as f:
            raw_data = f.read()
    else:
        raise FileExistsError

    #data_yaml = yaml.safe_load_all(raw_data)
	ryaml = YAML(typ='safe')
    data_yaml = ryaml.load_all(raw_data)

    file_data = []
    for data in data_yaml:
        if data != None:

            row = []
            for k, v in data.items():
                if file_type == "joint_state":
                    if (k == "position") or (k == "velocity") or (k == "effort"):
                        row.append(v)

                elif file_type == "object_pose":
                    if k == "pose":
                        for data in v:
                            row.append(data["position"]["x"])
                            row.append(data["position"]["y"])
                            row.append(data["position"]["z"])
                            row.append(data["orientation"]["x"])
                            row.append(data["orientation"]["y"])
                            row.append(data["orientation"]["z"])
                            row.append(data["orientation"]["w"])
                    elif k == "twist":
                        for data in v:
                            row.append(data["linear"]["x"])
                            row.append(data["linear"]["y"])
                            row.append(data["linear"]["z"])
                            row.append(data["angular"]["x"])
                            row.append(data["angular"]["y"])
                            row.append(data["angular"]["z"])

            file_data.append(np.array(row).flatten().tolist())

    return file_data


def _down_sampling(input_2d_list, step=10):
    """
    @description  :  下采样1000hz的oject到与joint数据相同的100hz
    ---------
    @param  :
    -------
    @Returns  :  采样结果
    -------
    """
    res = []
    for i in range(0, len(input_2d_list), step):
        res.append(input_2d_list[i])
    return res


def _list_align(input_2dlist, target_length):
    """
    @description  :  对齐长度不同的joint和object文件
    ---------
    @param  :  input_2dlist -> list[][]
    ---------
    @Returns  :
    ---------
    """
    diff = len(input_2dlist) - target_length
    step = target_length // (diff + 1)
    for i in range(diff):
        input_2dlist.pop(i + step)
    return input_2dlist


def data_preproprecess(joint_2d_list, object_2d_list):
    """
    @description  : down sampling object data, align joint and object data, merge
    ---------
    @param joint_2d_list : fetch joint data
    @param object_2d_list : fetch object data
    -------
    @Returns _2d_list : preprocessed np.array
    -------
    """
    # 下采样
    jf = joint_2d_list
    of = _down_sampling(object_2d_list)

    # 对齐数据
    jf_length = len(jf)
    of_length = len(of)
    if jf_length > of_length:
        jf = _list_align(jf, of_length)
    elif jf_length < of_length:
        of = _list_align(of, jf_length)

    # 合并数据
    arr = np.concatenate(
        (
            np.array(jf, dtype=float),
            np.array(of, dtype=float),
        ),
        axis=1,
    )
    return arr


def _select_features(np_arr, symbol_headers):
    if np_arr.shape[1] == 123:
        df = pd.DataFrame(np_arr, columns=FETCH_DATA_HEADERS_123)
        return df[symbol_headers].values
    elif np_arr.shape[1] == 91:
        df = pd.DataFrame(np_arr, columns=FETCH_VALID_HEADERS_91)
        return df[symbol_headers].values
    else:
        raise RuntimeError("feature dimension error")


def z_score_normalization(x, mean, std):
    return (x - mean) / (std + 1e-5)


def check_symbol(joint_file_path, object_file_path, action):

    # 读取mean std

    #z_score_mean = np.load("/home/fdse/robot_learning/generate_classifier/z_score_mean.npy")
    #z_score_std = np.load("/home/fdse/robot_learning/generate_classifier/z_score_std.npy")
    # 读取 yaml 数据
    rospy.loginfo("Get Environment Data ...")
    # joint_data = read_yaml_data(joint_file_path, "joint_state")
    # object_data = read_yaml_data(object_file_path, "object_pose")

    # 数据预处理，得到维度123的环境数据
    # print("数据预处理")
    # env_data = data_preproprecess(joint_2d_list=joint_data, object_2d_list=object_data)
    # if env_data.shape[0] >= 350:
    #     env_data = env_data[-350:]
    # else:
    #     print("Length < 350")
    #     pass
    # np.save("test/symprop_app_test_env_data.npy", env_data)

    # env_data = np.load("test/symprop_app_test_env_data.npy")
    payload = {'joint_data_path':joint_file_path, 'object_data_path':object_file_path}
    url = "http://127.0.0.1:5001/read_yaml/"
    response = requests.request(method="POST", url=url, data=json.dumps(payload), timeout=(5000, 5000))
    env_data = json.loads(response.text)['env_data']

    rospy.loginfo("Checking Propositions ...")
    url = "http://127.0.0.1:5001/prediction/baseline/mlp/"
    # url = "http://127.0.0.1:5001/prediction/reptile/"
    # url = "http://127.0.0.1:5001/prediction/maml/"
    payload = {"action": action, "data": env_data}
    response = requests.request(method="POST", url=url, data=json.dumps(payload), timeout=(5000, 5000))
    rospy.loginfo("Checking Result: {}".format(response.text))
    rospy.loginfo('')
    return response.text
    # 读取symbol
    # with open("./generate_classifier/symbols2.json", "r") as f:
    #     motion_symbol = json.load(f)

    # check_res = {}
    #     # for motion in motion_symbol:

    # for symbol in motion_symbol[motion]:
    #     pred_X = _select_features(env_data, FETCH_VALID_HEADERS_91)
    #     pred_x = z_score_normalization(pred_X,mean_std_dict[symbol][0], mean_std_dict[symbol][1] )

    #     pred_X = _select_features(pred_x, motion_symbol[motion][symbol]).reshape(1, -1)
    #     clf = _read_from_pickle(os.path.join("generate_classifier", (symbol + ".joblib")))
    #     pred_y = clf.predict(pred_X)
    #     check_res[symbol] = pred_y[0]

    # sorted_symbols = sorted(check_res, key=lambda k: int(k[-1]))

    # res = []
    # for symbol in sorted_symbols:
    #     res.append(check_res[symbol])
    # print(sorted_symbols, res)

    # return res


if __name__ == '__main__':
    rospy.init_node('rosplan_interface_movebase', anonymous=False)
    rpmb = RPMoveBasePy()
    rpmb.runActionInterface()

