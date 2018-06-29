#!/usr/bin/env python
import select
import sys
import itertools
import thread
import functools
import collections
import pickle
import operator
import os, os.path

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

from action_builders import *

def shutdown(tree):
    """Stop the tree."""
    tree.interrupt()

def printMediaDir():
    print "media/"
    for file in os.listdir('media'):
        if file == os.listdir('media')[-1]:
            print u" \u2514\u2500%s" % file
        else:
            print u" \u251c\u2500%s" % file

class LastActionVisitor(py_trees.visitors.VisitorBase):
    def __init__(self, lfd):
        self.full = False
        self.lfd = lfd

    def run(self, behav):
        # If an action was run and finished then push it to the last actions
        if behav.name in self.lfd.action_indices.keys():
            if behav.status != py_trees.common.Status.RUNNING:
                self.lfd.last_actions.append(self.lfd.action_indices[behav.name])
                # Update world state
                world_state = self.lfd.get_state()
                self.lfd.write_state(world_state)

class LfD:
    def __init__(self):
        # Init ros
        # Initilise the node
        rospy.init_node('lfd')

        # Demonstrations
        self.demo_states = None
        self.demo_actions = None
        self.redo_states = None
        self.redo_actions = None
        # self.redo_last_actions = None

        # Model
        self.clf = tree.DecisionTreeClassifier()
        self.model_file = 'media/model.sav'

        # Actions
        self.actions = {
            0: Action('extend_torso', BuildFullyExtendTorso),
            1: Action('mid_torso', BuildMoveTorsoBehavior),
            2: Action('pick', BuildPickBehavior),
            3: Action('detect_centroid', BuildCentroidDetectorBehavior),
            4: Action('tuck', BuildTuckWithCondBehavior),
            5: Action('place', BuildPlaceBehavior),
            6: Action('look_strait', BuildHeadMoveBehavior),
        }
        self.action_names = {}
        self.action_indices = {}
        for key, value in self.actions.iteritems():
            self.action_names[key] = str(value)
            self.action_indices[str(value)] = key
        num_last_actions = 3
        self.last_actions = collections.deque(maxlen=num_last_actions)
        for _ in xrange(num_last_actions):
            self.last_actions.append(0)

        # Blackboard setup
        self.blackboard = py_trees.blackboard.Blackboard()
        print 'Run a bunch of actions to init blackboard and robot'
        self.run_action('detect_centroid')
        self.run_action('extend_torso')
        self.run_action('tuck')
        self.run_action('look_strait')
        print 'Blackboard:', self.blackboard
        print 'Actions:\n\t', self.action_names.values()

        # Features setup
        self.feature_names = (['Cx', 'Cy', 'Cz', 'Csuccess']
            + ['LA' + str(num_last_actions - i) for i in xrange(self.last_actions.maxlen)])
        print 'Features:\n\t', self.feature_names

        # Build a perception tree
        # self.build_perception_tree()

    def run_action(self, action_id):
        # If the action_id is a name then get the key
        try:
            action_id = self.action_indices[action_id]
        except:
            pass
        # Run the action
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
        self.tree = self.dt_to_bt(self.clf)
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

    def execute_dt(self):
        state = self.get_state()
        print 'World state is:'
        self.print_state(state)
        print 'Path through tree:\n', self.clf.decision_path(state)
        action_id = self.clf.predict(state)[0]
        print 'Resulting action is: ' + self.action_names[action_id]
        self.run_action(action_id)

    def execute(self):
        state = self.get_state()
        self.write_state(state)
        print 'World state is:'
        self.print_state(state)
        self.tree.tick()

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

    def write_state(self, state):
        for name, value in zip(self.feature_names, state[0]):
            self.blackboard.set(name, value)

    def print_state(self, state):
        for name, value in zip(self.feature_names, state[0]):
            print '{0}: {1}'.format(name, value)

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
                    '\n\tRedo last undone(rd)'
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
                        state = 'LoadModel'
                        # self.clf = pickle.load(open(self.model_file, 'rb'))
                        # self.render_model()
                        # state = 'AskUser'
                    elif user_input == 'w':
                        state = 'WriteModel'
                        # pickle.dump(self.clf, open(self.model_file, 'wb'))
                        # state = 'AskUser'
                    elif user_input == 'u':
                        if self.demo_states is not None and len(self.demo_states) != 0:
                            if self.redo_states is not None and len(self.redo_states) != 0:
                                self.redo_states = numpy.append(self.redo_states, [self.demo_states[-1]], axis=0)
                                self.redo_actions = numpy.append(self.redo_actions, [self.demo_actions[-1]])
                                # self.redo_last_actions = numpy.append(self.redo_last_actions, [self.last_actions[-1]])
                            else:
                                self.redo_states = [self.demo_states[-1]]
                                self.redo_actions = [self.demo_actions[-1]]
                                # self.redo_last_actions = [self.last_actions[-1]]
                            self.demo_states = self.demo_states[:-1]
                            self.demo_actions = self.demo_actions[:-1]
                            # self.last_actions.pop()
                        else:
                            print "No demonstration to undo."
                        state = 'AskUser'
                    elif user_input == 'rd':
                        if self.redo_states is not None and len(self.redo_states) != 0:
                            # print self.demo_states
                            # print self.redo_states
                            # print self.redo_states[-1]
                            self.demo_states = numpy.append(self.demo_states, [self.redo_states[-1]], axis=0)
                            self.demo_actions = numpy.append(self.demo_actions, [self.redo_actions[-1]])
                            # self.last_actions.append(self.redo_last_actions[-1])
                            self.redo_states = self.redo_states[:-1]
                            self.redo_actions = self.redo_actions[:-1]
                            # self.redo_last_actions = self.redo_last_actions[:-1]
                        else:
                            print "No undone demonstartion to redo."
                        state = 'AskUser'
                    else:
                        state = 'AskUser'
            elif state == 'Demonstrate':
                print 'Demonstrate'
                self.redo_states = None
                self.redo_actions = None
                world_state = self.get_state()
                print 'World state is'
                self.print_state(world_state)
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
                    self.tree.interrupt()
                    # Interrupting the tree may not allow the last action to be written appropriatly
                    raw_input()
                    state = "AskUser"
            elif state == 'Rc':
                print 'In RC mode select an action to perform or type anything else to exit'
                user_input = raw_input('What action should be taken ' + str(self.action_names) + ': ')
                try:
                    self.run_action(int(user_input))
                except Exception as e:
                    state = 'AskUser'
            elif state == "LoadModel":
                printMediaDir()
                try:
                    user_input = raw_input('Filename: ')
                    full_path = 'media/' + user_input
                    if os.path.isfile(full_path):
                        self.clf = pickle.load(open(full_path, 'rb'))
                        self.render_model()
                    else:
                        print "Error: file does not exist!"
                    state = 'AskUser'
                except Exception as e:
                    print 'Error:', e
                    state = 'AskUser'
                    continue
            elif state == "WriteModel":
                printMediaDir()
                try:
                    user_input = raw_input('Filename (.sav type recommended): ')
                    full_path = 'media/' + user_input
                    if not os.path.isfile(full_path):
                        print "File does not exist. Creating...",
                        open(full_path, 'wb').close()
                        print "Done!"
                    else:
                        print "Warning: File already exists! Old version will be stored as %s.old" % user_input
                        if os.path.isfile(full_path + '.old'):
                            os.remove(full_path + '.old')
                        os.rename(full_path, full_path + '.old')
                    pickle.dump(self.clf, open(full_path, 'wb'))
                    state = 'AskUser'
                except Exception as e:
                    print 'Error:', e
                    state = 'AskUser'
                    continue

            # Sleep if needed
            sleep_rate.sleep()

    def dt_to_bt(self, dt):
        """Convert a decision tree to a behavior tree."""
        # Build the root
        #   The root is a sequence node that first writes the current state to the blackboard then
        #       Runs the behavior tree
        root = py_trees.composites.Sequence(name='root')
        # state_to_bb = StateToBB(name='state_to_bb')
        # root.add_child(state_to_bb)

        # Build the tree from dt.tree_
        #   The decision tree is stored in a few arrays of size node_count
        #   children_left - the child to the left (true) of the node
        #   children_right - the child to the right (false) of the current node
        #   threshold - the value to threshold on feat <= thres
        #   feature - the feature to split on
        #   value - the node_count by n_outputs=1, max_n_classes array containing the output
        def build_bt(dt, node_id, parent):
            """Recursive function for buiding the BT."""

            # Get info from dt
            left_child = dt.tree_.children_left[node_id]
            right_child = dt.tree_.children_right[node_id]

            # Check if leaf node or decision node
            if left_child == -1 or right_child == -1:
                # Leaf node
                class_index = dt.classes_[numpy.argmax(dt.tree_.value[node_id][0])]
                name = self.action_names[class_index]
                action = self.actions[class_index].get_builder()(name)
                parent.add_child(action)
            else:
                # Decision node
                #                      ?
                #        +-------------|------------+
                #       ->                         ->
                #     +--|-------+               +--|-------+
                #   cond   true_sub_tree       !cond  false_sub_tree
                # Get info from dt
                feature_name = self.feature_names[dt.tree_.feature[node_id]]
                thres = dt.tree_.threshold[node_id]

                # Define nodes
                sel = py_trees.composites.Selector('sel_' + str(node_id))
                seq_true = py_trees.composites.Sequence('seq_true_' + str(node_id))
                seq_false = py_trees.composites.Sequence('seq_false_' + str(node_id))
                cond_true = py_trees.blackboard.CheckBlackboardVariable(
                    name='cond_{0}_{1}<={2}'.format(node_id, feature_name, thres),
                    variable_name=feature_name,
                    expected_value=thres,
                    comparison_operator=operator.le
                )
                cond_false = py_trees.blackboard.CheckBlackboardVariable(
                    name='cond_{0}_{1}>{2}'.format(node_id, feature_name, thres),
                    variable_name=feature_name,
                    expected_value=thres,
                    comparison_operator=operator.gt
                )

                # Construct tree
                parent.add_child(sel)
                sel.add_children([seq_true, seq_false])
                seq_true.add_child(cond_true)
                seq_false.add_child(cond_false)
                build_bt(dt, left_child, seq_true)
                build_bt(dt, right_child, seq_false)


        build_bt(dt, 0, root)

        tree = py_trees_ros.trees.BehaviourTree(root)
        tree.visitors.append(LastActionVisitor(self))
        tree.setup(30)
        rospy.on_shutdown(functools.partial(shutdown, tree))

        return tree


def main():
    os.chdir(os.path.expanduser('~/catkin_ws/src/mobile_manipulation/lfd'))

    lfd = LfD()
    lfd.run()

if __name__ == '__main__':
    main()
