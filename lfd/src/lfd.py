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
from behavior_manager.interfaces.manipulation_behavior import FullyExtendTorso, ColapseTorso, MoveTorsoBehavior, PickBehavior, TuckWithCondBehavior, PlaceBehavior
from behavior_manager.interfaces.centroid_detector_behavior import CentroidDetectorBehavior

class SimpleTree:
    def __init__(self, behav):
        self.behav = behav
        self.behav.setup(timeout=30)

    def run_once(self):
        self.behav.tick_once()
        while self.behav.status == py_trees.Status.RUNNING:
            self.behav.tick_once()

    def __str__(self):
        return self.behav.name

class LfD:
    def __init__(self):
        # Init ros
        # Initilise the node
        rospy.init_node('lfd')

        # Demonstrations
        self.demo_states = None
        self.demo_actions = None

        # Actions
        self.actions = {
            0: SimpleTree(FullyExtendTorso('extend_torso')),
            1: SimpleTree(MoveTorsoBehavior('mid_torso', 0.2)),
            2: SimpleTree(PickBehavior('pick')),
            3: SimpleTree(CentroidDetectorBehavior('detect_centroid')),
            4: SimpleTree(TuckWithCondBehavior('tuck', 1)),
            5: SimpleTree(PlaceBehavior('place')),
        }
        self.action_names = {}
        for key, value in self.actions.iteritems():
            self.action_names[key] = str(value)

        # Blackboard setup
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set('detect_centroid/min_x', 0)
        self.blackboard.set('detect_centroid/max_x', 1.2)
        self.blackboard.set('detect_centroid/min_y', 0)
        self.blackboard.set('detect_centroid/max_y', 0.6)
        self.blackboard.set('detect_centroid/min_z', 0.8)
        self.blackboard.set('detect_centroid/max_z', 0.9)
        self.blackboard.set('shelf', String('shelf2'))
        print 'Run a bunch of actions to init blackboard cause I am lazy'
        self.actions[3].run_once()
        self.actions[0].run_once()
        self.actions[4].run_once()
        print 'State:', self.blackboard

    # A model that takes in a state and produces an action
    def model(self, state):
        return 0

    # Produces a model from states
    def learn(self, states, actions):
        clf = LinearSVC()
        clf.fit(states, actions)
        return clf.predict

    def execute(self):
        state = self.get_state()
        print 'World state is:\n' + str(self.blackboard)
        action_id = self.model(state)[0]
        print 'Resulting action is: ' + self.action_names[action_id]
        self.actions[action_id].run_once()

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
                print 'Demonstrate(d), learn(l), execute(e), rc(r):'
                state = 'WaitForUser'
            elif state == 'WaitForUser':
                if select.select([sys.stdin,], [], [], 0.0)[0]:
                    user_input = raw_input()
                    if user_input == 'd':
                        state = 'Demonstrate'
                    elif user_input == 'l':
                        self.model = self.learn(self.demo_states, self.demo_actions)
                        state = 'AskUser'
                    elif user_input == 'e':
                        state = 'Execute'
                    elif user_input == 'r':
                        state = 'Rc'
                    else:
                        state = 'AskUser'
            elif state == 'Demonstrate':
                print 'Demonstrate'
                world_state = self.get_state()
                print 'World state is\n' + str(self.blackboard)
                user_input = numpy.array([[int(
                    raw_input('What action should be taken ' + str(self.action_names) + ': ')
                )]])

                # Perform action
                self.actions[user_input[0,0]].run_once()

                # Save the state action combo
                if self.demo_states is not None:
                    self.demo_states = numpy.append(self.demo_states, world_state, axis=0)
                    self.demo_actions = numpy.append(self.demo_actions, user_input)
                else:
                    self.demo_states = world_state
                    self.demo_actions = user_input
                state = 'AskUser'
            elif state == 'Execute':
                print 'Press enter to end execution'
                self.execute()
                if select.select([sys.stdin,], [], [], 0.0)[0]:
                    raw_input()
                    state = "AskUser"
            elif state == 'Rc':
                print 'In RC mode select an action to perform or type anything else to exit'
                user_input = raw_input('What action should be taken ' + str(self.action_names) + ': ')
                try:
                    self.actions[int(user_input)].run_once()
                except Exception as e:
                    state = 'AskUser'

            # Sleep if needed
            sleep_rate.sleep()


def main():
    lfd = LfD()
    lfd.run()

if __name__ == '__main__':
    main()
