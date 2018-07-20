"""Decision tree to behavior tree.

This file contains the code for converting decision trees to behavior trees.
There are two methods that can be used.
TODO(...): Explain methods.

"""
from copy import deepcopy
import itertools
import re
from sympy.logic import SOPform

def dt_to_min_sop(true_children, false_children, clf):
    """Convert a decision tree to a minimum sum of product representation.

    inputs:
        true_children: TODO(Allen).
        false_children: TODO(Allen).
        clf: TODO(Allen).
    outputs:
        TODO(Allen).
    conventions:
    	1. In dt, for a decision node, the true path is encoded as
    	a positive int. Otherwise, a negative int.
    	2. The root node should be the first element in input list.
    	3. The index for each node should be encoded as a positive int.
    	So, the root node should be encoded as 1.
    """
    path_dict = {}
    num_class = 0
    num_decision = 0
    decision_symbols = []
    for x in range(len(true_children)):
    	if true_children[x] != -1 and false_children[x] != -1:
    		num_decision += 1
    		decision_symbols.append('a' + str(x))

    def dfs(node_id, prev_path):
    	if true_children[node_id] != -1 and false_children[node_id] != -1:
    		# Inner node
    		prev_path.append((node_id, True))
    		dfs(true_children[node_id], deepcopy(prev_path))
    		prev_path[-1] = (node_id, False)
    		dfs(false_children[node_id], deepcopy(prev_path))
    	else:
    		# Leaf node
    		class_index = clf[node_id]
    		if class_index in path_dict:
    			path_dict[class_index].append(prev_path)
    		else:
    			path_dict[class_index] = [prev_path]

    def parse_symbol_expr(bool_expr, class_index):
    	# WARNING: May be problematic.
    	simplified_path = []
    	products = re.split('\|', bool_expr)
    	for partial_product in products:
    		one_product = []
    		symbols = re.split('\&', partial_product)
    		for x in symbols:
    			x = x.strip()
    			x = x.strip('()')
    			if x[0] == '~':
    				x = int(x[2:])
    				one_product.append((x, False))
    			else:
    				x = int(x[1:])
    				one_product.append((x, True))
    		simplified_path.append(one_product)
    	path_dict[class_index] = deepcopy(simplified_path)


    # Get the paths for each leaf node
    dfs(0, [])
    num_class = len(path_dict)

    # For debugging
    # print path_dict
    # print "num_class: " + str(num_class)
    # print "num_decision: " + str(num_decision)
    # print decision_symbols

    # Simplify each class boolean expression using sympy
    for key in path_dict:
    	# Assumption: The path list for each class has been sorted
    	# based on absolute values
    	if len(path_dict[key]) > 1:
    		minterm = []
    		for line in itertools.product([0, 1], repeat=num_decision):
    			line = list(line)
    			# For each line, check whether it is the minterm or not
    			minterm_flag = False
    			for single_path in path_dict[key]:
    				one_product_flag = True
    				for bit in single_path:
    					bit_symbol = 'a' + str(bit[0])

    					if line[decision_symbols.index(bit_symbol)] == 1 and bit[1] == False:
    						one_product_flag = False
    						break
    					elif line[decision_symbols.index(bit_symbol)] == 0 and bit[1] == True:
    						one_product_flag = False
    						break
    				if one_product_flag:
    					minterm_flag = True
    					break
    			if minterm_flag:
    				minterm.append(line)

    		# print minterm
    		bool_expr = str(SOPform(decision_symbols, minterm))
    		# print bool_expr
    		parse_symbol_expr(bool_expr, key)

    # print path_dict
    return path_dict

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

    # If there is only one minterm in min_sop, omit fallback node.
    if len(min_sop) == 1:
        # If there is only one condition node in the minterm, omit sequence node.
        if len(min_sop[0]) == 1:
            cond = min_sop[0][0]
            cond_node = BTNode(
                'cond_{0}{1}'.format('' if cond[1] else '~', cond[0]),
                BTNode.CONDITION,
                user_id=cond[0],
                neg_cond=not cond[1]
            )
            return cond_node
        else:
            and_node = BTNode('and', BTNode.SEQUENCE)
            minterm = min_sop[0]
            for cond in minterm:
                cond_node = BTNode(
                    'cond_{0}{1}'.format('' if cond[1] else '~', cond[0]),
                    BTNode.CONDITION,
                    user_id=cond[0],
                    neg_cond=not cond[1]
                )
                and_node.add_child(cond_node)
            return and_node


    or_node = BTNode('or', BTNode.FALLBACK)
    for minterm in min_sop:
        # If there is only one condition node in minterm, omit sequence node.
        if len(minterm) == 1:
            cond = minterm[0]
            cond_node = BTNode(
                'cond_{0}{1}'.format('' if cond[1] else '~', cond[0]), 
                BTNode.CONDITION, 
                user_id=cond[0], 
                neg_cond=not cond[1]
            )
            or_node.add_child(cond_node)
        else:
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
        cond = min_sop_to_bt_cond(sop)
        act = BTNode('act_{0}'.format(clf), BTNode.ACTION, user_id=clf)
        # Build tree
        # If the return cond node is a seq node, omit seq node here
        if cond.node_type == BTNode.SEQUENCE:
            cond.name = 'seq'
            cond.add_child(act)
            root.add_child(cond)
        else:
            seq = BTNode('seq', BTNode.SEQUENCE)
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
        return BTNode('act_{0}'.format(clf[node_id]), BTNode.ACTION, user_id=clf[node_id])
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
