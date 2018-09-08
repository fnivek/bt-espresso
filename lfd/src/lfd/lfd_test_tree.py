#!/usr/bin/env python

import sys
import functools

import rospy
import py_trees
import py_trees_ros
from lfd import GetDusterOrientation

def build_tree():
    root = py_trees.composites.Sequence(name='test_nav_behaviors_and_conds')
    root.add_children([
      BuildGetFeatherDusterOrientation('feather_orientation'),
      BuildGetMFCDusterOrientation('mfc_orientation'),
    ])
    return py_trees_ros.trees.BehaviourTree(root)

def shutdown(tree):
    """Stop the tree."""
    tree.interrupt()

def main():
    """
    Entry point for the demo script.
    """
    # Init ros
    rospy.init_node("lfd_test_tree")

    # This is a test tree so initilize set debug level
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # Write to the blackboard a navigation msg
    blackboard = py_trees.blackboard.Blackboard()

    # Build a tree
    tree = build_tree()

    # Make sure the tree gets shutdown when ros does
    rospy.on_shutdown(functools.partial(shutdown, tree))

    # Setup the tree
    #   Give it 10 seconds to start
    timeout = 10
    print 'Waiting %ds for tree to start' % (timeout,)
    if not tree.setup(timeout=timeout):
        rospy.logerr("failed to setup the tree, aborting.")
        sys.exit(1)

    # Start ticking tree at 1hz
    tree.tick_tock(1000)

if __name__ == '__main__':
    main()
