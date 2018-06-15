#!/usr/bin/env python
import select
import sys
import itertools

from sklearn.svm import LinearSVC
import numpy

import rospy
import actionlib
from std_msgs.msg import String

import py_trees

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction
from behavior_manager.interfaces.manipulation_behavior import FullyExtendTorso, ColapseTorso, MoveTorsoBehavior, PickBehavior
from behavior_manager.interfaces.centroid_detector_behavior import CentroidDetectorBehavior

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

        # Demonstrations
        self.states = None
        self.actions = None

        # Actions
        self.extend_torso = SimpleTree(FullyExtendTorso('extend_torso'))
        self.mid_torso = SimpleTree(MoveTorsoBehavior('mid_torso', 0.2))
        self.pick = SimpleTree(PickBehavior('pick'))
        self.detect_centroid = SimpleTree(CentroidDetectorBehavior('detect_centroid'))

        # Blackboard setup
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set('detect_centroid/min_x', 0)
        self.blackboard.set('detect_centroid/max_x', 1.5)
        self.blackboard.set('detect_centroid/min_y', -0.5)
        self.blackboard.set('detect_centroid/max_y', 0.5)
        self.blackboard.set('detect_centroid/min_z', 0.6)
        self.blackboard.set('detect_centroid/max_z', 2)
        self.blackboard.set('shelf', String('shelf2'))
        print 'Detecting a centroid to write to init blackboard cause I am lazy'
        self.detect_centroid.run_once()
        print 'State:', self.blackboard

    # A model that takes in a state and produces an action
    def model(self, state):
        return 'h'

    # Produces a model from states
    def learn(self, states, actions):
        clf = LinearSVC()
        clf.fit(states, actions)
        return clf.predict

    def execute(self):
        self.detect_centroid.run_once()
        state = self.get_state()
        print 'World state is:\n' + str(self.blackboard)
        action = self.model(state)[0]
        print 'Resulting action is: ' + action
        if action == 'h':
            self.extend_torso.run_once()
        else:
            self.mid_torso.run_once()

    def get_state(self):
        return numpy.array([[
            self.blackboard.centroid.centroid.position.x,
            self.blackboard.centroid.centroid.position.y,
            self.blackboard.centroid.centroid.position.z,
            self.blackboard.centroid.centroid.orientation.x,
            self.blackboard.centroid.centroid.orientation.y,
            self.blackboard.centroid.centroid.orientation.z,
            self.blackboard.centroid.centroid.orientation.w,
            self.blackboard.centroid.success
        ]])

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
                        state = 'Execute'
                    else:
                        state = 'AskUser'
            elif state == 'Demonstrate':
                print 'Demonstrate'
                self.detect_centroid.run_once()
                world_state = self.get_state()
                print 'World state is\n' + str(self.blackboard)
                user_input = numpy.array([[raw_input('What action should be taken; say hi (h) or nop (n):')]])
                if self.states is not None:
                    self.states = numpy.append(self.states, world_state, axis=0)
                    self.actions = numpy.append(self.actions, user_input)
                else:
                    self.states = world_state
                    self.actions = user_input
                state = 'AskUser'
            elif state == 'Execute':
                print 'Press enter to end execution'
                self.execute()
                if select.select([sys.stdin,], [], [], 0.0)[0]:
                    raw_input()
                    state = "AskUser"

            # Sleep if needed
            sleep_rate.sleep()


def main():
    lfd = LfD()
    lfd.run()

if __name__ == '__main__':
    main()
