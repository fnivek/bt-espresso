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

from sklearn.feature_selection import SelectPercentile, f_classif
from sklearn.pipeline import Pipeline
from sklearn import tree
import numpy

import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import navigation_msgs.msg

import py_trees
import py_trees_ros

from action_builders import *
import dt_to_bt


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
        if behav.status != py_trees.common.Status.RUNNING:
            if behav in self.lfd.last_action_dict:
                self.lfd.last_actions.append(self.lfd.last_action_dict[behav])
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
        self.clf = None
        self.model_file = 'media/model.sav'
        self.tree = None

        # Enumerate the type of BT
        NAIVE = 'naive'
        SOP = 'SOP'
        CDNF = 'CDNF'
        ESPRESSO = 'espresso'
        self.bt_mode = None

        # Robot-Specific Config
        self.joint_names = [
            "l_wheel_joint",
            "r_wheel_joint",
            "torso_lift_joint",
            "bellows_joint",
            "head_pan_joint",
            "head_tilt_joint",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "upperarm_roll_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
            "l_gripper_finger_joint",
            "r_gripper_finger_joint",
        ]

        # Dict for converting item name to index as feature
        # TODO(Allen): Make a global dictionary in other places
        self.item_name_to_index_dict = {
            'feather': 0,
            'handle': 1,
            'mfc_top': 2,
            'mfc_bottom': 3
        }

        # Actions
        actions = [
          Action('update_joints', BuildUpdateJointsBehavior),
          Action('detect_centroid', BuildCentroidDetectorBehavior),
          Action('detect_hand', BuildDetectHandHandBehavior),
          Action('detect_person', BuildPersonDetectorBehavior),
          Action('get_feather_duster_orientation', BuildGetFeatherDusterOrientation),
          Action('get_mfc_orientation', BuildGetMFCDusterOrientation),
          Action('extend_torso', BuildFullyExtendTorso),
          Action('mid_torso', BuildMoveTorsoBehavior),
          Action('tuck', BuildTuckWithCondBehavior),
          Action('tuck_unkown3', BuildTuckWithCondBehavior, 'unknown_3'),
          Action('pick_centroid', BuildPickBehavior),
          Action('pick_anything', BuildPickAnythingBehavior),
          Action('pick_obj', BuildPickObjBehavior),
          Action('set_target_feather', BuildSetObjDetectorTarget, 'feather'),
          Action('set_target_handle', BuildSetObjDetectorTarget, 'handle'),
          Action('set_target_mfc_top', BuildSetObjDetectorTarget, 'mfc_top'),
          Action('set_target_mfc_bottom', BuildSetObjDetectorTarget, 'mfc_bottom'),
          Action('place', BuildPlaceBehavior),
          Action('look_strait', BuildHeadMoveBehavior),
          Action('look_at_person', BuildLookAtPersonBehavior),
          Action('relative forward', BuildRelativeMoveBehavior, amp=0.5, direction='forward'),
          Action('nav_to_home', BuildNavBehavior, param_server_name='/swag_delivery/pose_home'),
          Action('nav_to_item1', BuildNavBehavior, param_server_name='/swag_delivery/pose_item1'),
          Action('nav_to_item2', BuildNavBehavior, param_server_name='/swag_delivery/pose_item2'),
          Action('nav_to_item3', BuildNavBehavior, param_server_name='/swag_delivery/pose_item3'),
          Action('nav_to_item4', BuildNavBehavior, param_server_name='/swag_delivery/pose_item4'),
          Action('nav_to_item5', BuildNavBehavior, param_server_name='/swag_delivery/pose_item5'),
          Action('nav_to_item6', BuildNavBehavior, param_server_name='/swag_delivery/pose_item6'),
          Action('say hello', BuildTTSBehavior, text='hello'),
          Action('say select item', BuildTTSBehavior, text='Please place your hand over the desired item'),
          Action('say item 1', BuildTTSBehavior, text='You have selected item 1'),
          Action('say item 2', BuildTTSBehavior, text='You have selected item 2'),
          Action('say item 3', BuildTTSBehavior, text='You have selected item 3'),
          Action('say Here is your item', BuildTTSBehavior, text='Here is your item'),
          Action('sleep', BuildSleepBehavior),
          Action('open_gripper', BuildOpenGripperBehavior),
          Action('close_gripper', BuildCloseGripperBehavior),
          # Action('detect handle', BuildBagDetectBehavior),
          # Action('grab handle', BuildBagGrabBehavior),
          # Action('grasploc', BuildGrasplocBehavior),
          # Action('grasploc_pick', BuildGrasplocPickBehavior),
          # Action('object_detector', BuildObjectDetectorBehavior)
        ]
        self.actions = dict(enumerate(actions))
        self.action_names = {}
        self.action_indices = {}
        for key, value in self.actions.iteritems():
            self.action_names[key] = str(value)
            self.action_indices[str(value)] = key
        num_last_actions = 10
        self.last_actions = collections.deque(maxlen=num_last_actions)
        for _ in xrange(num_last_actions):
            self.last_actions.append(0)

        # Blackboard setup
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set('duster_orientation', 0)
        print 'Run a bunch of actions to init blackboard and robot'
        self.run_action('detect_centroid')
        self.run_action('detect_hand')
        self.run_action('detect_person')
        self.run_action('extend_torso')
        self.run_action('tuck')
        self.run_action('look_strait')
        self.run_action('set_target_handle')
        while True:
            self.run_action('update_joints')
            fail = False
            for name in self.joint_names:
                if self.blackboard.get(name + "_position") is None:
                    print "Do not yet have value for %s_position" % name
                    fail = True
                    break
            if not fail:
                break

        # print 'Blackboard:', self.blackboard
        print 'Actions:\n\t', self.action_names.values()

        # Features setup
        self.feature_names = (
              ['person_detected', 'centroid_detected', 'hand_detected', 'hand_y', 'item', 'duster_orientation']
              # TODO(Kevin): Develop a way to allow indices of last actions to be different for different models
            + ['LA' + str(num_last_actions - i) for i in xrange(self.last_actions.maxlen)]
            + [name + "_position" for name in self.joint_names]
            + [name + "_velocity" for name in self.joint_names]
            + [name + "_effort" for name in self.joint_names]
            + ['at_home', 'at_item1', 'at_item2', 'at_item3', 'at_item4', 'at_item5', 'at_item6', 'arm_tucked', 'arm_2ed', 'arm_unknown_3ed', 'arm_4ed', 'arm_5ed', 'arm_6ed', 'arm_7ed'])
        self.retained_feats_names = list(self.feature_names)
        print 'Features:\n\t', self.feature_names

        # Build a perception tree
        self.start_perception_tree()

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
        get_joint_states = BuildUpdateJointsBehavior(name='get_joint_states')
        at_home = BuildAtPoseBehavior(name='at_home', param_server_name='/swag_delivery/pose_home')
        at_item1 = BuildAtPoseBehavior(name='at_item1', param_server_name='/swag_delivery/pose_item1')
        at_item2 = BuildAtPoseBehavior(name='at_item2', param_server_name='/swag_delivery/pose_item2')
        at_item3 = BuildAtPoseBehavior(name='at_item3', param_server_name='/swag_delivery/pose_item3')
        at_item1 = BuildAtPoseBehavior(name='at_item4', param_server_name='/swag_delivery/pose_item4')
        at_item2 = BuildAtPoseBehavior(name='at_item5', param_server_name='/swag_delivery/pose_item5')
        at_item3 = BuildAtPoseBehavior(name='at_item6', param_server_name='/swag_delivery/pose_item6')
        arm_tucked = BuildArmTuckedBehavior(name='arm_tucked', tuck_pose='tuck')
        arm_2ed = BuildArmTuckedBehavior(name='arm_2ed', tuck_pose=2)
        arm_unknown_3ed = BuildArmTuckedBehavior(name='arm_unknown_3ed', tuck_pose='unknown_3')
        arm_4ed = BuildArmTuckedBehavior(name='arm_4ed', tuck_pose=4)
        arm_5ed = BuildArmTuckedBehavior(name='arm_5ed', tuck_pose=5)
        arm_6ed = BuildArmTuckedBehavior(name='arm_6ed', tuck_pose=6)
        arm_7ed = BuildArmTuckedBehavior(name='arm_7ed', tuck_pose=7)
        arm_look_at_objed = BuildArmTuckedBehavior(name='arm_look_at_objed', tuck_pose='look_at_obj')

        # Define structure of tree
        root.add_children([
            get_joint_states,
            at_home,
            at_item1,
            at_item2,
            at_item3,
            arm_tucked,
            arm_2ed,
            arm_unknown_3ed,
            arm_4ed,
            arm_5ed,
            arm_6ed,
            arm_7ed,
            arm_look_at_objed,
        ])

        return root

    def shutdown_perception_tree(self, tree):
        """Stop the tree."""
        tree.interrupt()
        tree.blackboard_exchange.unregister_services()

    def stop_perception_tree(self):
        """Stop the tree."""
        self.perception_tree.interrupt()
        self.perception_tree.blackboard_exchange.unregister_services()

    def start_perception_tree(self):
        """Start a perception tree."""
        root = self.build_perception_tree()

        # Wrap tree in ros tree
        self.perception_tree = py_trees_ros.trees.BehaviourTree(root)

        # Add a interupt hook and setup
        rospy.on_shutdown(functools.partial(self.shutdown_perception_tree, self.perception_tree))
        self.perception_tree.setup(30)

        # Start a thread for the tree
        thread.start_new_thread(self.perception_tree.tick_tock, (10,))

    # A model that takes in a state and produces an action
    def model(self, state):
        return 0

    def render_model(self):
        if self.tree is not None:
            self.stop_bt()
            self.tree = None
            self.bt_mode = None
        dot_data = tree.export_graphviz(
            self.clf.named_steps['classification'],
            out_file=None,
            filled=True,
            feature_names=self.retained_feats_names,
            class_names=[action_name for action_name in self.clf.classes_],
            impurity=False)
        graph = graphviz.Source(dot_data)
        graph.render('media/tree')

    # Produces a model from states
    def learn(self, states, actions):
        # print states
        # print "\n\n\n"
        # print actions

        self.clf = Pipeline([
          ('feature_selection', SelectPercentile(f_classif, percentile=100)),
          ('classification', tree.DecisionTreeClassifier(
            # max_depth=15,
            class_weight='balanced',
            # min_samples_leaf=2,
            )),
        ])
        self.clf.fit(states, actions)

        # Print retained features
        support_int = self.clf.named_steps['feature_selection'].get_support(True)
        self.retained_feats_names = [self.feature_names[i] for i in support_int]
        # for index in support_int:
        #     self.retained_feats_names.append(self.feature_names[index])
        print 'retained features:', self.retained_feats_names

        self.render_model()

    def execute_dt(self):
        # TODO(Kevin): Update state
        state = self.get_state()
        print 'World state is:'
        self.print_state(state)
        print 'Path through tree:\n', self.clf.decision_path(state)
        action_id = self.clf.predict(state)[0]
        print 'Resulting action is: ' + self.action_names[action_id]
        self.run_action(action_id)
        # TODO(Kevin): Write last action to last action list

    def execute(self):
        if self.tree == None:
            self.tree = self.get_bt(self.clf.named_steps['classification'])
        state = self.get_state()
        self.write_state(state)
        # print 'World state is:'
        # self.print_state(state)
        self.tree.tick()

    def get_state(self):
        # WARNING: Not thread safe!!!!
        #   Blackboard access is being made in the perception tree thread
        return numpy.array([
          [
            self.blackboard.detect_person.success,
            self.blackboard.detect_centroid.success,
            self.blackboard.detect_hand.success,
            self.blackboard.detect_hand.centroid.position.y,
            self.item_name_to_index_dict[self.blackboard.get('item')],
            self.blackboard.get('duster_orientation'),
          ]
        + list(self.last_actions)
        + [self.blackboard.get(name + "_position") for name in self.joint_names]
        + [self.blackboard.get(name + "_velocity") for name in self.joint_names]
        + [self.blackboard.get(name + "_effort") for name in self.joint_names]
        + [self.blackboard.get(name) for name in [
            'at_home',
            'at_item1',
            'at_item2',
            'at_item3',
            'at_item4',
            'at_item5',
            'at_item6',
            'arm_tucked',
            'arm_2ed',
            'arm_unknown_3ed',
            'arm_4ed',
            'arm_5ed',
            'arm_6ed',
            'arm_7ed',]]
        ])
        # return numpy.array([list(self.last_actions)])

    def write_state(self, state):
        for name, value in zip(self.feature_names, state[0]):
            self.blackboard.set(name, value)

    def print_state(self, state):
        print self.state_to_str(state)

    def state_to_str(self, state):
        state_str = ''
        for name, value in zip(self.feature_names, state[0]):
            state_str += '{0}: {1}\n'.format(name, value)
        state_str = state_str[:-1]
        return state_str


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
                    '\n\tQuit(q)'
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
                    elif user_input == 'q':
                        user_input = raw_input('Are you sure you want to quit? (y/n) ')
                        if user_input == 'y':
                            return
                        else:
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

    def construct_bt(self, struct, dt):
        """Convert a bt structure into a real bt.

        Inputs:
            struct: stucture of the bt in dt_to_bt.BTNode form.
            dt: The scipy decision tree structure
        Outputs:
            bt: A behavior tree

        """
        # Build this bt node
        bt = None
        if struct.node_type == dt_to_bt.BTNode.FALLBACK:
            bt = py_trees.composites.Selector(struct.name)
        if struct.node_type == dt_to_bt.BTNode.PARALLEL:
            bt = py_trees.composites.Parallel(struct.name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, synchronize=True, allow_failure=True)
        if struct.node_type == dt_to_bt.BTNode.SEQUENCE:
            bt = py_trees.composites.Sequence(struct.name)
        if struct.node_type == dt_to_bt.BTNode.ACTION:
            name = struct.user_id
            index = self.action_indices[struct.user_id]
            action = self.actions[index]
            bt = action.get_builder()(name, *action.builder_args, **action.builder_kwargs)
            self.last_action_dict[bt] = index
        if struct.node_type == dt_to_bt.BTNode.CONDITION:
            # Get info from dt
            feature_name = self.retained_feats_names[dt.tree_.feature[struct.user_id]]
            thres = dt.tree_.threshold[struct.user_id]

            # Negate if needed
            name = 'cond_{0}_{1}{2}{3:.3f}'
            op = operator.le
            op_str = '<='
            if struct.neg_cond:
                op = operator.gt
                op_str = '>'

            # Build condition
            bt = py_trees.blackboard.CheckBlackboardVariable(
                name=name.format(struct.user_id, feature_name, op_str, thres),
                variable_name=feature_name,
                expected_value=thres,
                comparison_operator=op
            )

        # Construct all its children
        for child in struct.children:
            child_bt = self.construct_bt(child, dt)
            bt.add_child(child_bt)

        return bt

    def get_bt(self, dt, bt_type='Espresso'):
        """Convert a decision tree to a behavior tree."""
        # Build the root
        #   The root is a sequence node that first writes the current state to the blackboard then
        #       Runs the behavior tree
        root = self.build_perception_tree()

        # Build the tree from dt.tree_
        #   The decision tree is stored in a few arrays of size node_count
        #   children_left - the child to the left (true) of the node
        #   children_right - the child to the right (false) of the current node
        #   threshold - the value to threshold on feat <= thres
        #   feature - the feature to split on
        #   value - the node_count by n_outputs=1, max_n_classes array containing the output
        #   TODO(Kevin): When using balanced weighting all values are binary either 0 or the same >0 float
        clf = [
            dt.classes_[numpy.argmax(dt.tree_.value[node_id][0])]
            for node_id in xrange(len(dt.tree_.children_left))
        ]
        if bt_type == 'Naive':
            bt_struct = dt_to_bt.simple_dt_to_bt(
                true_children=dt.tree_.children_left,
                false_children=dt.tree_.children_right,
                clf=clf
            )
        elif bt_type == 'SOP':
            bt_struct = dt_to_bt.dt_to_bt(
                true_children=dt.tree_.children_left,
                false_children=dt.tree_.children_right,
                clf=clf,
                bt_type='SOP'
            )
        elif bt_type == 'CDNF':
            bt_struct = dt_to_bt.dt_to_bt(
                true_children=dt.tree_.children_left,
                false_children=dt.tree_.children_right,
                clf=clf,
                bt_type='CDNF'
            )
        elif bt_type == 'Espresso':
            bt_struct = dt_to_bt.dt_to_bt(
                true_children=dt.tree_.children_left,
                false_children=dt.tree_.children_right,
                clf=clf,
                bt_type='Espresso'
            )
        self.bt_mode = bt_type

        # Make a dict to point to the root of each behavior we want to show up in last actions
        self.last_action_dict = {}

        root.add_child(self.construct_bt(bt_struct, dt))

        tree = py_trees_ros.trees.BehaviourTree(root)
        tree.visitors.append(LastActionVisitor(self))
        tree.setup(60)
        rospy.on_shutdown(functools.partial(shutdown, tree))

        return tree

    def stop_bt(self):
        if self.tree is not None:
            self.tree.interrupt()
            self.tree.blackboard_exchange.unregister_services()


def main():
    os.chdir(os.path.expanduser('~/catkin_ws/src/mobile_manipulation/lfd'))

    lfd = LfD()
    lfd.run()

if __name__ == '__main__':
    main()
