#!/usr/bin/env python3.4
"""Decision tree to behavior tree.

This file contains the code for converting decision trees to behavior trees
using Espresso heuristic method.
TODO(...): Explain methods.

"""
from pyeda.inter import *
from pyeda.boolalg.expr import *
import threading
import re
import argparse
import sys
from copy import deepcopy

class EspressoSOPThread(threading.Thread):

    def __init__(self, name, path_dict, key, decision_symbols):
        threading.Thread.__init__(self)
        self.name = name
        self.path_dict = path_dict
        self.key = key
        self.decision_symbols = decision_symbols

    def run(self):
        sum_of_product_term = None
        for single_path in self.path_dict[self.key]:
            product_term = None
            for bit in single_path:
                if bit[1] == True:
                    if product_term == None:
                        product_term = self.decision_symbols[bit[0]]
                    else:
                        product_term = And(product_term, self.decision_symbols[bit[0]])
                else:
                    if product_term == None:
                        product_term = Not(self.decision_symbols[bit[0]])
                    else:
                        product_term = And(product_term, Not(self.decision_symbols[bit[0]]))
            if sum_of_product_term == None:
                sum_of_product_term = product_term
            else:
                sum_of_product_term = Or(sum_of_product_term, product_term)

        bool_expr, = espresso_exprs(sum_of_product_term.to_dnf())
        parse_symbol_expr_espresso(str(bool_expr), self.key, self.path_dict)



def parse_symbol_expr_espresso(bool_expr, class_index, path_dict):
    # WARNING: May be problematic.
    simplified_path = []
    if bool_expr[0] == '~':
        simplified_path.append([(int(bool_expr[2:]), False)])
    elif bool_expr[0] == 'a':
        simplified_path.append([(int(bool_expr[1:]), True)])
    elif bool_expr[0:2] == 'Or':
        bool_expr = bool_expr[3:(len(bool_expr) - 1)]
        and_terms = re.findall(r'And\((.+?)\)', bool_expr)
        single_terms = re.split('\,', re.sub(r'And\((.+?)\)', '', bool_expr))
        for x in single_terms:
            x = x.strip()
            if x != '' and x[0] == '~':
                simplified_path.append([(int(x[2:]), False)])
            elif x != '' and x[0] == 'a':
                simplified_path.append([(int(x[1:]), True)])
        for each_product in and_terms:
            each_product_term = []
            symbols = re.split('\,', each_product)
            for x in symbols:
                x = x.strip()
                if x[0] == '~':
                    each_product_term.append((int(x[2:]), False))
                else:
                    each_product_term.append((int(x[1:]), True))
            simplified_path.append(each_product_term)
    elif bool_expr[0:3] == 'And':
        one_product = []
        bool_expr = bool_expr.lstrip('And(')
        bool_expr = bool_expr.rstrip(')')
        symbols = re.split('\,', bool_expr)
        for x in symbols:
            x = x.strip()
            if x[0] == '~':
                one_product.append((int(x[2:]), False))
            else:
                one_product.append((int(x[1:]), True))
        simplified_path.append(one_product)
    path_dict[class_index] = deepcopy(simplified_path)

def main():
    true_children = []
    false_children = []
    clf = []
    
    length = int((len(sys.argv) - 1) / 3)
    for x in range(length):
        true_children.append(int(sys.argv[1 + x]))
    for x in range(length, 2 * length):
        false_children.append(int(sys.argv[1 + x]))
    for x in range(2 * length, 3 * length):
        clf.append(int(sys.argv[1 + x]))

    path_dict = {}
    decision_symbols = {}
    thread_pool = []
    for x in range(len(true_children)):
        # If this is an inner node or decision node.
        if true_children[x] != -1 and false_children[x] != -1:
            name = 'a' + str(x)
            decision_symbols[x] = exprvar(name)

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

    # Get the paths for each leaf node
    dfs(0, [])

    # Simplify each class boolean expression using sympy
    for key in path_dict:
        # Assumption: The path list for each class has been sorted
        # based on absolute values
        if len(path_dict[key]) > 1:
            sop_thread = EspressoSOPThread('espresso_thread_' + str(key), path_dict, key, decision_symbols)
            thread_pool.append(sop_thread)
            sop_thread.start()

    # Wait for all the threads' finishing
    for thr in thread_pool:
        thr.join()

    print(str(path_dict))

if __name__ == '__main__':
	main()
