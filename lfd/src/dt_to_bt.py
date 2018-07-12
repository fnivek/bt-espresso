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
        fals_children: TODO(Allen).
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
    print path_dict
    print "num_class: " + str(num_class)
    print "num_decision: " + str(num_decision)
    print decision_symbols

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

    		print minterm
    		bool_expr = str(SOPform(decision_symbols, minterm))
    		print bool_expr
    		parse_symbol_expr(bool_expr, key)

    print path_dict