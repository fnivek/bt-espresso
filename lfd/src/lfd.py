#!/usr/bin/env python
import select
import sys

from sklearn.svm import LinearSVC
import numpy

import rospy
import actionlib

import py_trees

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction
from behavior_manager.interfaces.manipulation_behavior import FullyExtendTorso, ColapseTorso, MoveTorsoBehavior

class SimpleTree:
    def __init__(self, behav):
        self.behav = behav
        self.behav.setup(timeout=30)

    def run_once(self):
        self.behav.tick_once()
        while self.behav.status == py_trees.Status.RUNNING:
            self.behav.tick_once()

class LfD:
    def __init__(self):
        # Init ros
        # Initilise the node
        rospy.init_node('lfd')

        # Centroid detector
        self.goal = DetectCentroidGoal()
        self.goal.min_x = 0
        self.goal.max_x = 1.5
        self.goal.min_y = -0.5
        self.goal.max_y = 0.5
        self.goal.min_z = 0.6
        self.goal.max_z = 2
        self.client = actionlib.SimpleActionClient('/centroid_detector', DetectCentroidAction)
        rospy.loginfo('Waiting for centroid detector action server')
        self.client.wait_for_server()
        rospy.loginfo('Centroid detector action server is running')

        # Demonstrations
        self.states = None
        self.actions = None

        # Actions
        self.hi = SimpleTree(FullyExtendTorso('hi'))
        self.bye = SimpleTree(MoveTorsoBehavior('Bye', 0.2))

    # A model that takes in a state and produces an action
    def model(self, state):
        return 'h'

    # Produces a model from states
    def learn(self, states, actions):
        clf = LinearSVC()
        clf.fit(states, actions)
        return clf.predict

    def detect_centroid(self):
        # Send a goal to centroid detector
        rospy.loginfo('Detecting centroid')
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        # Convert result to a numpy array
        result = numpy.array([
            [
                result.centroid.position.x,
                result.centroid.position.y,
                result.centroid.position.z,
                result.centroid.orientation.x,
                result.centroid.orientation.y,
                result.centroid.orientation.z,
                result.centroid.orientation.w,
                result.success
            ]
        ])
        return result

    def execute(self):
        state = self.detect_centroid()
        print 'World state is:\n' + str(state)
        action = self.model(state)[0]
        print 'Resulting action is: ' + action
        if action == 'h':
            self.hi.run_once()
        else:
            self.bye.run_once()

    def run(self):
        # State
        state = 'AskUser'

        # Loop every 10hz
        sleep_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if state == 'AskUser':
                print 'Demonstrate(d), learn(l), execute(e):'
                state = 'WaitForUser'
            elif state == 'WaitForUser':
                if select.select([sys.stdin,], [], [], 0.0)[0]:
                    user_input = raw_input()
                    if user_input == 'd':
                        state = 'Demonstrate'
                    elif user_input == 'l':
                        self.model = self.learn(self.states, self.actions)
                        state = 'AskUser'
                    elif user_input == 'e':
                        self.execute()
                        state = 'AskUser'
                    else:
                        state = 'AskUser'
            elif state == 'Demonstrate':
                print 'Demonstrate'
                result = self.detect_centroid()
                print 'World state is\n' + str(result)
                user_input = numpy.array([[raw_input('What action should be taken; say hi (h) or nop (n):')]])
                if self.states is not None:
                    self.states = numpy.append(self.states, result, axis=0)
                    self.actions = numpy.append(self.actions, user_input)
                else:
                    self.states = result
                    self.actions = user_input
                state = 'AskUser'

            # Sleep if needed
            sleep_rate.sleep()


def main():
    lfd = LfD()
    lfd.run()

if __name__ == '__main__':
    main()
