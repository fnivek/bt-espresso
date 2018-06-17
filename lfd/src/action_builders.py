import py_trees
import py_trees_ros

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction
from behavior_manager.interfaces.manipulation_behavior import FullyExtendTorso, ColapseTorso, MoveTorsoBehavior, PickBehavior, TuckWithCondBehavior, PlaceBehavior
from behavior_manager.interfaces.centroid_detector_behavior import CentroidDetectorBehavior

class Action:
    def __init__(self, name, builder):
        self.name = name
        self.builder = builder
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
    return PickBehavior(name)
def BuildCentroidDetectorBehavior(name):
    return CentroidDetectorBehavior(name)
def BuildTuckWithCondBehavior(name):
    return TuckWithCondBehavior(name, 1)
def BuildPlaceBehavior(name):
    return PlaceBehavior(name)
