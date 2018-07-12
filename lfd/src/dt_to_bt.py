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

    Params:
        name: A name for the node
        node_type: the type of behavior tree node (see enumerated types bellow)
        children: a list of children
        user_id: an identifier provided by the user that indicates the condition or the action,
            associated with the node, this will be none for flow control nodes
        neg_cond: True a condition node should be left as is, False a condition node should be,
            negated

    """
    # Enumerate the types of nodes
    FALLBACK = 'fallback'
    SEQUENCE = 'sequence'
    ACTION = 'action'
    CONDITION = 'condition'

    def __init__(self, name='', node_type=None, children=None, user_id=None, neg_cond=False):
        self.name = name
        self.node_type = node_type
        if children is None:
            self.children = []
        self.user_id = user_id
        self.neg_cond = neg_cond

    def add_child(self, child):
        self.children.append(child)

    def __str__(self):
        text = '{0}({1}{2})\n'.format(
            self.name,
            '~' if self.neg_cond else '',
            self.node_type
        )
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
            cond_node = BTNode(
                'cond_{0}{1}'.format('' if cond[1] else '~', cond[0]),
                BTNode.CONDITION,
                user_id=cond[0],
                neg_cond=not cond[1]
            )
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
        act = BTNode('act_{0}'.format(clf), BTNode.ACTION, user_id=clf)
        # Build tree
        seq.add_child(cond)
        seq.add_child(act)
        root.add_child(seq)

    return root

def dt_to_bt(true_children, false_children, clf):
    """Convert a decision tree to a minimum sum of product representation.

    Inputs:
        true_children: TODO(Allen).
        false_children: TODO(Allen).
        clf: TODO(Allen).
    Outputs:
        TODO(Allen).

    """
    min_sops = dt_to_min_sop(true_children, false_children, clf)
    bt = min_sops_to_bt(min_sops)
    return bt


def simple_dt_to_bt(true_children, false_children, clf, node_id=0):
    """Convert a decision tree to a behavior tree.

    This is a recursive algorithm that directly converts the structures in a
    decision tree into a behavior tree.

    Inputs:
        true_children: TODO(Allen).
        false_children: TODO(Allen).
        clf: TODO(Allen).
        node_id: the current node id in the decision tree to be processing,
            this should be 0 to start from the root
    Outputs:
        TODO(Allen).

    """

    # Get info from dt
    true_child = true_children[node_id]
    right_child = false_children[node_id]

    # Check if leaf node or decision node
    if true_child == -1 or right_child == -1:
        return BTNode('act_{0}'.format(clf[node_id]), BTNode.ACTION)
    else:
        # Decision node
        #                      ?
        #        +-------------|------------+
        #       ->                         ->
        #     +--|-------+               +--|-------+
        #   cond   true_sub_tree       !cond  false_sub_tree

        # Define nodes
        fall = BTNode('fall', BTNode.FALLBACK)
        seq_true = BTNode('seq_true', BTNode.SEQUENCE)
        seq_false = BTNode('seq_false', BTNode.SEQUENCE)
        cond_true = BTNode('cond_{0}'.format(node_id), BTNode.CONDITION, user_id=node_id)
        cond_false = BTNode('cond_~{0}'.format(node_id), BTNode.CONDITION, user_id=node_id, neg_cond=True)

        # Construct tree
        fall.add_child(seq_true)
        fall.add_child(seq_false)
        seq_true.add_child(cond_true)
        seq_true.add_child(
            simple_dt_to_bt(true_children, false_children, clf, true_children[node_id])
        )
        seq_false.add_child(cond_false)
        seq_false.add_child(
            simple_dt_to_bt(true_children, false_children, clf, false_children[node_id])
        )

        return fall
