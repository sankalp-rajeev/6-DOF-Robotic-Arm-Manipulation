#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from re import L
from shutil import move
from tabnanny import check
from tkinter import Y
# from turtle import color
# from six.moves import input
# from tf2_msgs import TFMessage
from tf import transformations
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

from turtle import position
import trajectory_msgs.msg
# from moveit_commander import (
#     RobotCommander,
#     PlanningSceneInterface,
#     roscpp_initialize,
#     roscpp_shutdown,
# )

check_bit = None


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

  

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -tau / 8
        # joint_goal[2] = 0
        # joint_goal[3] = -tau / 4
        # joint_goal[4] = 0
        # joint_goal[5] = tau / 6  # 1/6 of a turn
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        # move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def closedGripper(self, posture=trajectory_msgs.msg.JointTrajectory()):
        posture.joint_names.append('left_joint')
        posture.joint_names.append('right_joint')
        pt = trajectory_msgs.msg.JointTrajectoryPoint()
        pt.positions.append(0.0)
        pt.positions.append(0.0)
        pt.time_from_start = rospy.Duration(0.8)
        posture.points.append(pt)

    def openGripper(self,posture=trajectory_msgs.msg.JointTrajectory()):
        posture.joint_names.append('left_joint')
        posture.joint_names.append('right_joint')
        pt = trajectory_msgs.msg.JointTrajectoryPoint()
        pt.positions.append(0.03)
        pt.positions.append(-0.03)
        pt.time_from_start = rospy.Duration(0.5)
        posture.points.append(pt)
    

    def go_to_pose_goal1(self,x,y,z,roll,pitch,yaw):
        move_group = self.move_group
        current_pose = self.move_group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        q = transformations.quaternion_from_euler(roll,pitch,yaw)
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets()
        
    
    def go_to_pose_goal(self,x,y,z,xo,yo,zo,wo):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        # move_group = moveit_commander.MoveGroupCommander('G1')
        current_pose = self.move_group.get_current_pose().pose
        
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = xo
        pose_goal.orientation.y = yo
        pose_goal.orientation.z = zo
        pose_goal.orientation.w = wo
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        if all_close(pose_goal,current_pose, 0.03) == False:
            print('in condition')

            move_group.set_pose_target(pose_goal)

            ## Now, we call the planner to compute the plan and execute it.
            plan = move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            move_group.clear_pose_targets()

            ## END_SUB_TUTORIAL

            # For testing:
            # Note that since this section of code will not be included in the tutorials
            # we use the class variable rather than the copied state variable
            current_pose = self.move_group.get_current_pose().pose
            return all_close(pose_goal, current_pose, 0.01)
        else:
            pass

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL
    def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    # def attach_box(self, timeout=4):
    #     # Copy class variables to local variables to make the web tutorials more clear.
    #     # In practice, you should use the class variables directly unless you have a good
    #     # reason not to.
    #     box_name = self.box_name
    #     robot = self.robot
    #     scene = self.scene
    #     eef_link = self.eef_link
    #     group_names = self.group_names

    #     ## BEGIN_SUB_TUTORIAL attach_object
    #     ##
    #     ## Attaching Objects to the Robot
    #     ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    #     ## robot be able to touch them without the planning scene reporting the contact as a
    #     ## collision. By adding link names to the ``touch_links`` array, we are telling the
    #     ## planning scene to ignore collisions between those links and the box. For the Panda
    #     ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    #     ## you should change this value to the name of your end effector group name.
    #     grasping_group = "EF"
    #     touch_links = robot.get_link_names(group=grasping_group)
    #     scene.attach_box(eef_link, box_name, touch_links=touch_links)
    #     ## END_SUB_TUTORIAL

    #     # We wait for the planning scene to update.
    #     return self.wait_for_state_update(
    #         box_is_attached=True, box_is_known=False, timeout=timeout
    #     )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name='obj16')
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self,object ,timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(object)
        print('removed box')
        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
    
    def add_pick_box(self,x,y,z,xo,yo,zo,wo,timeout=4):
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        # q = transformations.quaternion_from_euler(0,0,0)
        box_pose.pose.orientation.x = xo
        box_pose.pose.orientation.y = yo
        box_pose.pose.orientation.z = zo
        box_pose.pose.orientation.w = wo
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        scene.add_box("obj16", box_pose, size=(0.01, 0.04, 0.02))
        
        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        # self.box_name = box_name
        # return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_base_box(self):
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose= moveit_msgs.msg.AttachedCollisionObject()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        # q = transformations.quaternion_from_euler(0,0,0)
        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = -0.001
        scene.add_box("base", box_pose, size=(2, 2, 0.0005))
        
    
    def pick(self,x,y,z,xo,yo,zo,wo):
    # pick an object
        grasp = moveit_msgs.msg.Grasp()
        
        grasp.grasp_pose.header.frame_id = '"base_link"'
        q_recv = transformations.euler_from_quaternion([xo,yo,zo,wo])
        print([xo,yo,zo,wo])
        # print(q_recv)
        pose_goal = geometry_msgs.msg.Pose()
        q = transformations.quaternion_from_euler(q_recv[0],q_recv[1]-1.5708,q_recv[2])
        # print(q)
        grasp.grasp_pose.pose.position.x = x
        grasp.grasp_pose.pose.position.y = y
        grasp.grasp_pose.pose.position.z = z
        grasp.grasp_pose.pose.orientation.x = q[0]
        grasp.grasp_pose.pose.orientation.y = q[1]
        grasp.grasp_pose.pose.orientation.z = q[2]
        grasp.grasp_pose.pose.orientation.w = q[3]

        
        grasp.pre_grasp_approach.direction.header.frame_id = 'base_link'
        grasp.pre_grasp_approach.direction.vector.x = -1
        grasp.pre_grasp_approach.min_distance = 0.08
        grasp.pre_grasp_approach.desired_distance = 0.09
        grasp.post_grasp_retreat.direction.vector.x = 1.0
        grasp.post_grasp_retreat.min_distance = 0.08
        grasp.post_grasp_retreat.desired_distance = 0.09
        
        self.openGripper(grasp.pre_grasp_posture)
        
        self.closedGripper(grasp.grasp_posture)
        
        self.move_group.pick('obj16',grasp)
        
        
    def place(self):
        # Placing an Object
        place = moveit_msgs.msg.PlaceLocation()
        place.place_pose.header.frame_id = "base_link"
        q1 = transformations.quaternion_from_euler(0,3.14159,0)
        place.place_pose.pose.position.x = 0.35
        place.place_pose.pose.position.y = -0.2
        place.place_pose.pose.position.z = 0.2
        place.place_pose.pose.orientation.x = q1[0]
        place.place_pose.pose.orientation.y = q1[1]
        place.place_pose.pose.orientation.z = q1[2]
        place.place_pose.pose.orientation.w = q1[3]
        
        # place.post_place_retreat.direction.header.frame_id = "base_link"
        place.pre_place_approach.direction.vector.x = 1
        place.pre_place_approach.min_distance = 0.08
        place.pre_place_approach.desired_distance = 0.09
        
        place.post_place_retreat.direction.vector.x = -1
        place.post_place_retreat.min_distance =0.09
        place.post_place_retreat.desired_distance = 0.1
        
        self.openGripper(place.post_place_posture)
        
        self.move_group.place('obj16',place)
        
    def init(self):
        move_group = self.move_group
        tutorial.detach_box()
        tutorial.remove_box('obj16')
        move_group.set_pose_target('init')
        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        


def tf2data(data):
        input('Press Enter to continue')
        tutorial.add_pick_box(data.transform.translation.x,data.transform.translation.y,data.transform.translation.z,data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w)
        tutorial.pick(data.transform.translation.x,data.transform.translation.y,data.transform.translation.z,data.transform.rotation.x,data.transform.rotation.y,data.transform.rotation.z,data.transform.rotation.w)
        tutorial.place()
        # tutorial.init()
        
        
        # print(data)

# def check(data):
#     if len(data.data) != 0:
#         print(len(data.data))
#         rospy.Subscriber("obj16_pose", geometry_msgs.msg.TransformStamped, tf2data,queue_size=1)
#     else:
#         pass
    
def set_pose1():
    try:
#         # arm = MoveGroupPythonInterfaceTutorial()
#         # rospy.init_node('subtfnode', anonymous=True)
        rospy.Subscriber("obj16_pose", geometry_msgs.msg.TransformStamped, tf2data,queue_size=1)
        # rospy.Subscriber('objects', std_msgs.msg.Float32MultiArray, check,queue_size=1)
#         # tutorial.go_to_pose_goal(data.transform.translation.x,data.transform.translation.y,data.transform.translation.z, data.transform.rotation.w)
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

tutorial = MoveGroupPythonInterfaceTutorial()
# tutorial.add_base_box()
# tutorial.add_box(0,0,-0.001,0,0,0,1,2,2,0.0005,'base')

if __name__ == "__main__":
    # tutorial.add_box()
    # tutorial.pick()
    # tutorial.place()
    
  # set_pose1()
    # tutorial.add_box(0.3,0.2,0.2,0,0,0,1,0.02,0.02,0.02,'obj16')
    # tutorial.open_gripper()
    # tutorial.go_to_pose_goal(0.3,0.2,0.2,0,0,0,1)
    
    # tutorial.close_gripper()
    # tutorial.pick('obj16',0.4,0.2,0.2,0,0,0,1)
    # tutorial.go_to_pose_goal(0.3,0.2,0.2,0,0,0,1)
    tutorial.go_to_pose_goal(0.3,0.2,0.2,0,0,0,1)

