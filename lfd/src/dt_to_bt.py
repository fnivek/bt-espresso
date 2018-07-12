"""Decision tree to behavior tree.

This file contains the code for converting decision trees to behavior trees.
There are two methods that can be used.
TODO(...): Explain methods.

"""
def dt_to_min_sop(true_children, false_children, clf):
    """Convert a decision tree to a minimum sum of product representation.

    inputs:
        true_children: TODO(Allen).
        false_children: TODO(Allen).
        clf: TODO(Allen).
    outputs:
        TODO(Allen).
    """
    pass

class BTNode:
    """BTNode.

    A simple structure to represent a node in a behavior tree.

    """
    # Enumerate the types of nodes
    FALLBACK = 'fallback'
    SEQUENCE = 'sequence'
    ACTION = 'action'
    CONDITION = 'condition'

    def __init__(self, name='', node_type=None, children=None):
        self.name = name
        self.node_type = node_type
        if children is None:
            self.children = []

    def add_child(self, child):
        self.children.append(child)

    def __str__(self):
        text = '{0}({1})\n'.format(self.name, self.node_type)
        for child in self.children:
            child_text = str(child)
            first = True
            for row in child_text.splitlines():
                if first:
                    text += '  |-->{0}\n'.format(row)
                    first = False
                else:
                    text += '  |  {0}\n'.format(row)

        return text


def min_sop_to_bt_cond(min_sop):
    """Convert a minimum sop to a behavior tree condition.

    Inputs:
        min_sop: a list of list of ints representing a sum of products,
            each int is a representation of a condition,
            each sublist is a minterm.
    Outputs:
        cond: a representation of the structure of a bt that implements the SoP.

    """
    or_node = BTNode('or', BTNode.FALLBACK)
    for minterm in min_sop:
        and_node = BTNode('and', BTNode.SEQUENCE)
        for cond in minterm:
            cond_node = BTNode('cond_{0}'.format(cond), BTNode.CONDITION)
            and_node.add_child(cond_node)
        or_node.add_child(and_node)

    return or_node

def min_sops_to_bt(min_sops):
    """Convert a list of minimum sum of products to a behavior tree.

    Inputs:
        min_sops: a dict with classification as the key and sop as the value,
            the key is an integer that represents the classification,
            the sop is a list of list of ints, each int represents a condition.
    Outputs:
        bt: the struture of a behavior tree,
            The structure is encoded in the BTNode class (above).

    """
    # Define the root
    root = BTNode('root', BTNode.FALLBACK)

    # Loop through all sops
    for clf, sop in min_sops.iteritems():
        # Define nodes
        seq = BTNode('seq', BTNode.SEQUENCE)
        cond = min_sop_to_bt_cond(sop)
        act = BTNode('act_{0}'.format(clf), BTNode.ACTION)
        # Build tree
        seq.add_child(cond)
        seq.add_child(act)
        root.add_child(seq)

    return root

def dt_to_bt(true_children, false_children, clf):
    """Convert a decision tree to a minimum sum of product representation.

    inputs:
        true_children: TODO(Allen).
        false_children: TODO(Allen).
        clf: TODO(Allen).
    outputs:
        TODO(Allen).

    """
    min_sops = dt_to_min_sop(true_children, false_children, clf)
    bt = min_sops_to_bt(min_sops)
    return bt
