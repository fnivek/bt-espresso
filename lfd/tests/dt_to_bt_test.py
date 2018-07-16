from lfd.dt_to_bt import *

# Test 1
# true_children = [1, -1, 3, 4, -1, -1, -1]
# false_children = [2, -1, 6, 5, -1, -1, -1]
# clf = [-1, 1, -1, -1, 1, 2, 3]
# min_sop = {
#     1: [[(0, True)], [(2, True), (3, True)]],
#     2: [[(0, False), (2, True), (3, False)]],
#     3: [[(0, False), (2, False)]]
# }

# Test 2
true_children = [1, 3, 5, -1, 7, -1, -1, -1, -1]
false_children = [2, 4, 7, -1, 8, -1, -1, -1, -1]
clf = [-1, -1, -1, 3, -1, 7, 7, 7, 8]
min_sop = {
    3: [[(0, True), (1, True)]],
    7: [[(0, False)], [(1, False), (4, True)]],
    8: [[(0, True), (1, False), (4, False)]]
}

# Test 3
# true_children = [1, 3, 5, 7, 9, -1, -1, -1, 11, 12, -1, -1, -1, -1, -1]
# false_children = [2, 4, 6, 8, 10, -1, -1, -1, 14, 13, -1, -1, -1, -1, -1]
# clf = [-1, -1, -1, -1, -1, 2, 1, 1, -1, -1, 1, 2, 1, 2, 2]
# min_sop = {
#     1: [[(0, True), (1, True), (3, True)], [(0, True), (1, False), (9, True)], [(0, True), (1, False), (4, False)], [(0, False), (2, False)]],
#     2: [[(0, True), (1, True), (3, False)], [(0, True), (1, False), (4, True), (9, False)], [(0, False), (2, True)]]
# }

# Test 4
# true_children = [1, 3, 5, 7, 9, -1, 11, -1, 13, -1, 15, -1, -1, -1, 17, -1, -1, -1, -1]
# false_children = [2, 4, 6, 8, 10, -1, 12, -1, 14, -1, 16, -1, -1, -1, 18, -1, -1, -1, -1]
# clf = [-1, -1, -1, -1, -1, 1, -1, 1, -1, 2, -1, 2, 3, 1, -1, 3, 1, 2, 3]
# min_sop = {
#     1: [[(0, True), (1, True), (3, True)], [(0, True), (1, True), (8, True)], [(0, True), (1, False), (4, False), (10, False)], [(0, False), (2, True)]],
#     2: [[(0, True), (1, True), (3, False), (8, False), (14, True)], [(0, True), (1, False), (4, True)], [(0, False), (2, False), (6, True)]],
#     3: [[(0, True), (1, True), (3, False), (8, False), (14, False)], [(0, True), (1, False), (4, False), (10, True)], [(0, False), (2, False), (6, False)]]
# }


def dt_to_min_sop_test():
    print 'Start dt_to_min_sop_test'
    min_sop = dt_to_min_sop(true_children, false_children, clf)
    print min_sop


def min_sop_to_bt_cond_test():
    print 'Start min_sop_to_bt_cond_test'
    for sop in min_sop.values():
        cond = min_sop_to_bt_cond(sop)
        print sop
        print cond


def min_sops_to_bt_test():
    print 'Start min_sops_to_bt_test'
    bt = min_sops_to_bt(min_sop)
    print bt


def dt_to_bt_test():
    print 'Start dt_to_bt_test'
    bt = dt_to_bt(true_children, false_children, clf)
    print bt


def simple_dt_to_bt_test():
    print 'Start simple_dt_to_bt_test'
    bt = simple_dt_to_bt(true_children, false_children, clf)
    print bt
