#!/usr/bin/env python
import rospy
import actionlib
import tf
from math import radians, atan2, cos, sin

from fetch_manipulation_pipeline.msg import GrabBagAction, GrabBagGoal
import py_trees
import py_trees_ros
from geometry_msgs.msg import Pose
from copy import deepcopy


class GrabBagBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, name, *argv, **kwargs):
        super(GrabBagBehavior, self).__init__(
            name=name,
            action_spec=fetch_manipulation_pipeline.msg.GrabBagAction,
            action_goal=fetch_manipulation_pipeline.msg.GrabBagGoal(),
            action_namespace='grab_bag',
            *argv,
            **kwargs
        )

    def initialise(self):
        super(GrabBagBehavior, self).initialise()
        # Get two points from blackboard
        blackboard = py_trees.blackboard.Blackboard()
        front_point = blackboard.get('front_handle')
        back_point = blackboard.get('back_handle')

        if front_point is None:
            rospy.logerr('handle points was not defined for grab bag behavior')
            self.action_goal = fetch_manipulation_pipeline.msg.GrabBagGoal()
            return

        # x1, y1 is the back handle. (x2, y2) is the front handle
        x1, y1, z1 = back_point
        x2, y2, z2 = front_point

        pose_ = Pose()

        # Calculate angles from the two points of the handles.
        x = 90
        y = atan2(x1-x2, y1-y2)
        z = 0

        # Calculate quaternions from the angles.
        quaternion = tf.transformations.quaternion_from_euler(radians(x), y, 0, 'rxyz')
        pose_.orientation.x = quaternion[0]
        pose_.orientation.y = quaternion[1]
        pose_.orientation.z = quaternion[2]
        pose_.orientation.w = quaternion[3]

        # Set grasp pose
        grasp_pose = deepcopy(pose_)
        # Set pre-grasp pose
        pre_grasp_pose = deepcopy(pose_);

        # Offset
        grasp_pose.position.x = x1 - 0.08 * cos(y)
        grasp_pose.position.y = y1 - 0.08 * cos(y)
        grasp_pose.position.z = (z1 + z2) / 2.0

        # Offset
        pre_grasp_pose.position.x = x2 - 0.25 * cos(y)
        pre_grasp_pose.position.y = y2 - 0.25 * sin(y)
        pre_grasp_pose.position.z = (z1 + z2) / 2.0


        # Set the action goal
        self.action_goal.grasp_pose = grasp_pose

        self.action_goal.pre_grasp_pose = pre_grasp_pose

        self.action_goal.cartesian_max_try = 3

        rospy.loginfo('Pick Goal Constructed.')
