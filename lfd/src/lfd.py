#!/usr/bin/env python
import select
import sys
import itertools
import thread
import functools
import collections
import pickle

import graphviz

from sklearn.svm import LinearSVC
from sklearn import tree
import numpy

import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import py_trees
import py_trees_ros

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

        # Model
        self.clf = tree.DecisionTreeClassifier()
        self.model_file = 'media/model.sav'

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
        num_actions = 1
        self.last_actions = collections.deque(maxlen=num_actions)
        for _ in xrange(num_actions):
            self.last_actions.append(0)

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
        self.run_action(3)
        self.run_action(0)
        self.run_action(4)
        print 'State:', self.blackboard
        print 'Actions:\n\t', self.action_names.values()

        # Features setup
        self.feature_names = (['Cx', 'Cy', 'Cz', 'Csuccess']
            + ['LA' + str(num_actions - i) for i in xrange(self.last_actions.maxlen)])
        print 'Features:\n\t', self.feature_names

        # Build a perception tree
        # self.build_perception_tree()

    def run_action(self, action_id):
        self.actions[action_id].run_once()
        self.last_actions.append(action_id)

    def build_perception_tree(self):
        # Define behaviors
        root = py_trees.composites.Parallel('perception_root')
        get_joint_states = py_trees_ros.subscribers.ToBlackboard('get_joint_states',
            '/joint_states',
            JointState,
            {'joint_states': None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
        )
        detect_centroid = CentroidDetectorBehavior('detect_centroid')

        # Define structure of tree
        root.add_children([
            get_joint_states,
            detect_centroid
        ])

        # Wrap tree in ros tree
        self.perception_tree = py_trees_ros.trees.BehaviourTree(root)

        # Add a interupt hook and setup
        rospy.on_shutdown(functools.partial(self.shutdown_perception_tree, self.perception_tree))
        self.perception_tree.setup(30)

        # Start a thread for the tree
        thread.start_new_thread(self.perception_tree.tick_tock, (10,))

    def shutdown_perception_tree(self, tree):
        """Stop the tree."""
        tree.interrupt()

    # A model that takes in a state and produces an action
    def model(self, state):
        return 0

    def render_model(self):
        dot_data = tree.export_graphviz(
            self.clf,
            out_file=None,
            filled=True,
            feature_names=self.feature_names,
            class_names=[self.action_names[action_id] for action_id in self.clf.classes_],
            impurity=False)
        graph = graphviz.Source(dot_data)
        graph.render('media/tree')

    # Produces a model from states
    def learn(self, states, actions):
        self.clf = tree.DecisionTreeClassifier()
        self.clf.fit(states, actions)
        self.render_model()

    def execute(self):
        state = self.get_state()
        print 'World state is:\n' + str(state)
        print 'Path through tree:\n', self.clf.decision_path(state)
        action_id = self.clf.predict(state)[0]
        print 'Resulting action is: ' + self.action_names[action_id]
        self.run_action(action_id)

    def get_state(self):
        # WARNING: Not thread safe!!!!
        #   Blackboard access is being made in the perception tree thread
        # TODO(Kevin): Figure out how to ensure I get the correct joint states
        return numpy.array([[
            self.blackboard.centroid.centroid.position.x,
            self.blackboard.centroid.centroid.position.y,
            self.blackboard.centroid.centroid.position.z,
            self.blackboard.centroid.success,
        ] # + list(self.blackboard.joint_states.position)
          # + list(self.blackboard.joint_states.velocity)
          # + list(self.blackboard.joint_states.effort)
          + list(self.last_actions)
        ])
        # return numpy.array([list(self.last_actions)])

    def run(self):
        # State
        state = 'AskUser'

        # Loop every 10hz
        sleep_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if state == 'AskUser':
                print ('Menu:'
                    '\n\tDemonstrate(d)'
                    '\n\tlearn(l)'
                    '\n\texecute(e)'
                    '\n\trc(r)'
                    '\n\tLoad model(lm)'
                    '\n\tWrite model(w)'
                    '\n\tUndo last demo(u)'
                    '\n')
                state = 'WaitForUser'
            elif state == 'WaitForUser':
                if select.select([sys.stdin,], [], [], 0.0)[0]:
                    user_input = raw_input()
                    if user_input == 'd':
                        state = 'Demonstrate'
                    elif user_input == 'l':
                        self.learn(self.demo_states, self.demo_actions)
                        state = 'AskUser'
                    elif user_input == 'e':
                        state = 'Execute'
                    elif user_input == 'r':
                        state = 'Rc'
                    elif user_input == 'lm':
                        self.clf = pickle.load(open(self.model_file, 'rb'))
                        self.render_model()
                        state = 'AskUser'
                    elif user_input == 'w':
                        pickle.dump(self.clf, open(self.model_file, 'wb'))
                        state = 'AskUser'
                    elif user_input == 'u':
                        if self.demo_states is not None and len(self.demo_states) != 0:
                            self.demo_states = self.demo_states[:-1]
                            self.demo_actions = self.demo_actions[:-1]
                        state = 'AskUser'
                    else:
                        state = 'AskUser'
            elif state == 'Demonstrate':
                print 'Demonstrate'
                world_state = self.get_state()
                print 'World state is\n' + str(world_state)
                try:
                    user_input = numpy.array([[int(
                        raw_input('What action should be taken ' + str(self.action_names) + ': ')
                    )]])
                except Exception as e:
                    print 'Error:', e
                    state = 'AskUser'
                    continue

                # Perform action
                self.run_action(user_input[0,0])

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
                    self.run_action(int(user_input))
                except Exception as e:
                    state = 'AskUser'

            # Sleep if needed
            sleep_rate.sleep()


def main():
    lfd = LfD()
    lfd.run()

if __name__ == '__main__':
    main()
