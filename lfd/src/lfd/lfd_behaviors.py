"""Learning From Demonstration behaviors

This file contains behaviors used in learning from demonstration.

"""
import py_trees_ros
import py_trees

from behavior_manager.interfaces.object_detector_behavior import ObjectDetectorBehavior

class GetFeatherDusterOrientation(py_trees.behaviour.Behaviour):
    def __init__(self, name, bounding_box_bb_key='obj_bb', duster_orientation_bb_key='duster_orientation'):
        super(GetFeatherDusterOrientation, self).__init__(name)
        self._bounding_box_bb_key = bounding_box_bb_key
        self._duster_orientation_bb_key = duster_orientation_bb_key

    def update(self):
        self.logger.debug("  %s [GetFeatherDusterOrientation::update()]" % self.name)
        # Get the left and right side of the bounding box
        blackboard = py_trees.blackboard.Blackboard()
        min_y = blackboard.get(self._bounding_box_bb_key+'/min_y')
        max_y = blackboard.get(self._bounding_box_bb_key+'/max_y')
        # Make sure a bounding box exist
        if min_y is None or max_y is None:
            return py_trees.Status.FAILURE
        # Check if the center x is on the right or left
        avg_y = (max_y + min_y) / 2.0
        print 'avg_y', avg_y
        y_value_divider = 250.0
        if avg_y > y_value_divider:
            blackboard.set(self._duster_orientation_bb_key, 0) # bottom
            print 'Feather bottom'
        else:
            blackboard.set(self._duster_orientation_bb_key, 1) # top
            print 'Feather top'

        return py_trees.Status.SUCCESS

def GetMFCDusterOrientation(name, bounding_box_bb_key='obj_bb', duster_orientation_bb_key='duster_orientation'):
    root = py_trees.composites.Selector(name)
    seq = py_trees.composites.Sequence(name)
    set_target = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name='duster_target', variable_value='mfc_top')
    top_detector = ObjectDetectorBehavior(name, target_bb_key='duster_target')
    set_oreintation_good = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name=duster_orientation_bb_key, variable_value=1)
    set_oreintation_bad = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name=duster_orientation_bb_key, variable_value=0)

    root.add_children([seq, set_oreintation_bad])
    seq.add_children([set_target, top_detector, set_oreintation_good])

    return root
