import py_trees
import py_trees_ros

from std_msgs.msg import String

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction
from behavior_manager.interfaces.manipulation_behavior import FullyExtendTorso, ColapseTorso, MoveTorsoBehavior, PickBehavior, TuckWithCondBehavior, PlaceBehavior
from behavior_manager.interfaces.centroid_detector_behavior import CentroidDetectorBehavior
from behavior_manager.interfaces.head_actuate_behavior import HeadMoveBehavior
from behavior_manager.interfaces.tts_behavior import TTSBehavior
from behavior_manager.interfaces.navigation_behavior import *

class Action:
    def __init__(self, name, builder, text=None, amp=None, direction=None):
        self.name = name
        self.builder = builder
        if text != None:
            self.action = builder(name, text)
        elif amp != None and direction != None:
            self.action = builder(name, amp, direction)
        else:
            self.action = builder(name)
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
    return TuckWithCondBehavior(name, 1)
def BuildPlaceBehavior(name):
    return PlaceBehavior(name)
def BuildHeadMoveBehavior(name):
    return HeadMoveBehavior(name, None, 1, 0, 0.5)
def BuildTTSBehavior(name, text='hello'):
    return TTSBehavior(name, text)
def BuildRelativeMoveBehavior(name, amp, direction):
    if direction == 'forward':
        return MoveForwardBehavior(name, amp)
    elif direction == 'backward':
        return MoveBackwardBehavior(name, amp)
    elif direction == 'right':
        return MoveRightBehavior(name, amp)
    elif direction == 'left':
        return MoveLeftBehavior(name, amp)
    elif direction == 'turn':
        return TurnInPlaceBehavior(name, amp)
    else:
        return None
