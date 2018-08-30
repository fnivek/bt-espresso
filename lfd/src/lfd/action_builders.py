import py_trees
import py_trees_ros

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction
from behavior_manager.interfaces.manipulation_behavior_new import FullyExtendTorso, MoveTorsoBehavior, PickBehavior, TuckWithCondBehavior, PlaceBehavior
from behavior_manager.interfaces.centroid_detector_behavior import CentroidDetectorBehavior
from behavior_manager.interfaces.head_actuate_behavior import HeadMoveBehavior
from behavior_manager.interfaces.tts_behavior import TTSBehavior
from behavior_manager.interfaces.update_joints_behavior import JointToBlackboardBehavior
from behavior_manager.interfaces.navigation_behavior import *
from behavior_manager.interfaces.detect_handles_behavior import DetectHandlesBehavior
from behavior_manager.interfaces.grab_bag_behavior import GrabBagBehavior

class Action:
    def __init__(self, name, builder, *builder_args, **builder_kwargs):
        self.name = name
        self.builder = builder
        self.builder_args = builder_args
        self.builder_kwargs = builder_kwargs
        self.action = builder(name, *builder_args, **builder_kwargs)
        self.action.setup(timeout=30)

    def run_once(self):
        self.action.tick_once()
        while self.action.status == py_trees.Status.RUNNING:
            self.action.tick_once()

    def __str__(self):
        return self.name

    def get_builder(self):
        return self.builder

    def get_name(self):
        return self.name

def BuildFullyExtendTorso(name):
    return FullyExtendTorso(name)
def BuildMoveTorsoBehavior(name):
    return MoveTorsoBehavior(name, 0.2)
def BuildPickBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set('shelf', String('shelf2'))
    return PickBehavior(name)
def BuildCentroidDetectorBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(name + '/min_x', 0)
    blackboard.set(name + '/max_x', 1.2)
    blackboard.set(name + '/min_y', 0)
    blackboard.set(name + '/max_y', 0.6)
    blackboard.set(name + '/min_z', 0.8)
    blackboard.set(name + '/max_z', 0.9)
    return CentroidDetectorBehavior(name)
def BuildTuckWithCondBehavior(name):
    return TuckWithCondBehavior(name, 'tuck')
def BuildPlaceBehavior(name):
    return PlaceBehavior(name)
def BuildHeadMoveBehavior(name):
    return HeadMoveBehavior(name, None, 1, 0, 0.5)
def BuildTTSBehavior(name, text='hello'):
    return TTSBehavior(name, text)
def BuildUpdateJointsBehavior(name):
    return JointToBlackboardBehavior(name=name, topic_name="/joint_states", topic_type=JointState)
def BuildRelativeMoveBehavior(name, amp=0.5, direction='forward'):
    if direction == 'forward':
        return NavOverride('override', MoveForwardBehavior(name, amp))
    elif direction == 'backward':
        return NavOverride('override', MoveBackwardBehavior(name, amp))
    elif direction == 'right':
        return NavOverride('override', MoveRightBehavior(name, amp))
    elif direction == 'left':
        return NavOverride('override', MoveLeftBehavior(name, amp))
    elif direction == 'turn':
        return NavOverride('override', TurnInPlaceBehavior(name, amp))
    else:
        return None
# Hardcoded behavior for grocery bag packing task
def BuildNavToBagBehavior(name):
    return NavOverride('override', NavToPoseWithCondBehavior(name=name, param_server_name='/grocery_bag_packing/pose_grocery_bag'))
def BuildNavToItemsBehavior(name):
    return NavOverride('override', NavToPoseWithCondBehavior(name=name, param_server_name='/grocery_bag_packing/pose_items'))
def BuildBagDetectBehavior(name):
    return DetectHandlesBehavior(name)
def BuildBagGrabBehavior(name):
    return GrabBagBehavior(name)