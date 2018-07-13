from lfd.dt_to_bt import *

true_children = [1, -1, 3, 4, -1, -1, -1]
false_children = [2, -1, 6, 5, -1, -1, -1]
clf = [-1, 1, -1, -1, 1, 2, 3]
min_sop = {
    1: [[(0, True)], [(2, True), (3, True)]],
    2: [[(0, False), (2, True), (3, False)]],
    3: [[(0, False), (2, False)]]
}

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
