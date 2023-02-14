#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import sys
from turtle import position
import rospy
import moveit_msgs.msg
import tf
from tf import transformations
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
import moveit_commander
import trajectory_msgs.msg
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    roscpp_initialize,
    roscpp_shutdown,
)
from geometry_msgs.msg import PoseStamped    
    
def closedGripper(posture=trajectory_msgs.msg.JointTrajectory()):
    """Specifies closed grasp posture for the pick and place motions"""
    # specify which end effector joints are involved in the grasp
    posture.joint_names.append('left_joint')
    posture.joint_names.append('right_joint')
    pt = JointTrajectoryPoint()

    # now specify where they should be positioned in order to 
    # close the hand
    pt.positions.append(0.0)
    pt.positions.append(0.0)
    # ... and how long it should take
    pt.time_from_start = rospy.Duration(0.8)
    posture.points.append(pt)

def openGripper(posture=trajectory_msgs.msg.JointTrajectory()):
    """Specifies open grasp posture for the pick and place motions"""
    # specify which end effector joints are involved in the grasp
    # posture = trajectory_msgs.msg.JointTrajectory()
    posture.joint_names.append('left_joint')
    posture.joint_names.append('right_joint')
    pt = JointTrajectoryPoint()

    # now specify where they should be positioned in order to 
    # open the hand
    pt.positions.append(0.04)
    pt.positions.append(-0.04)
    # ... and how it should take to open the hand
    pt.time_from_start = rospy.Duration(0.5)
    posture.points.append(pt)
        

if __name__ == "__main__":

    roscpp_initialize(sys.argv)
    rospy.init_node("moveit_py_demo", anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()

    rospy.sleep(1)
    

    # publish a demo scene
    # Adding a box
    o = PoseStamped()
    o.header.frame_id = robot.get_planning_frame()
    o.pose.position.x = 0.34
    o.pose.position.y = 0.2
    o.pose.position.z = 0.2
    # o.pose.position.x = 0.4
    # o.pose.position.y = -0.201
    # o.pose.position.z = 0.2
    q = transformations.quaternion_from_euler(0,0,0)
    print(q)
    o.pose.orientation.x = q[0]
    o.pose.orientation.y = q[1]
    o.pose.orientation.z = q[2]
    o.pose.orientation.w=q[3]
    scene.add_box("obj16", o, (0.03, 0.03, 0.03))

    rospy.sleep(1)

    
    # pick an object
    grasp = moveit_msgs.msg.Grasp()
    
    grasp.grasp_pose.header.frame_id = "base_link"
    q = transformations.quaternion_from_euler(0,1.5708,0)
    # q = transformations.quaternion_from_euler(-1.122, 1.552, -1.127)
    grasp.grasp_pose.pose.position.x =0.34
    grasp.grasp_pose.pose.position.y =0.2
    grasp.grasp_pose.pose.position.z =0.2
    # grasp.grasp_pose.pose.position.x =0.4
    # grasp.grasp_pose.pose.position.y =-0.201
    # grasp.grasp_pose.pose.position.z =0.2
    grasp.grasp_pose.pose.orientation.x = q[0]
    grasp.grasp_pose.pose.orientation.y = q[1]
    grasp.grasp_pose.pose.orientation.z = q[2]
    grasp.grasp_pose.pose.orientation.w = q[3]
    print (q[3])
    # grasp.pre_grasp_approach.direction.header.frame_id = 'object_16'
    grasp.pre_grasp_approach.direction.vector.x = 1
    grasp.pre_grasp_approach.min_distance = 0.095/2
    grasp.pre_grasp_approach.desired_distance = 0.115/2
    grasp.post_grasp_retreat.direction.vector.x = -1.0
    grasp.post_grasp_retreat.min_distance = 0.1
    grasp.post_grasp_retreat.desired_distance = 0.20
    
    
    openGripper(grasp.pre_grasp_posture)
    closedGripper(grasp.grasp_posture)
    
    robot.arm.pick('obj16',grasp)
    
    
    # Placing an Object
    place = moveit_msgs.msg.PlaceLocation()
    place.place_pose.header.frame_id = "base_link"
    q1 = transformations.quaternion_from_euler(0,-3.14159,0)
    # q1 = transformations.quaternion_from_euler(-1.122, 1.552, -1.127)
    place.place_pose.pose.position.x = -0.35
    place.place_pose.pose.position.y = 0.2
    place.place_pose.pose.position.z = 0.0
    # place.place_pose.pose.position.x = 0
    # place.place_pose.pose.position.y = -0.201
    # place.place_pose.pose.position.z = 0.2  
    place.place_pose.pose.orientation.x = q1[0]
    place.place_pose.pose.orientation.y = q1[1]
    place.place_pose.pose.orientation.z = q1[2]
    place.place_pose.pose.orientation.w = q1[3]
    
    # place.post_place_retreat.direction.header.frame_id = "base_link"
    place.pre_place_approach.direction.vector.z = -1
    place.pre_place_approach.min_distance = 0.095
    place.pre_place_approach.desired_distance = 0.115
    
    place.post_place_retreat.direction.vector.z = 1
    # place.post_place_retreat.direction.vector.y = 1

    place.post_place_retreat.min_distance =0.1
    place.post_place_retreat.desired_distance = 0.25
   
    
    openGripper(place.post_place_posture)
    
    robot.arm.place('obj16',place)
    
        
    # rospy.spin()
    roscpp_shutdown()    