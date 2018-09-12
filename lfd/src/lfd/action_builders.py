import py_trees
import py_trees_ros

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from centroid_detector_msgs.msg import DetectCentroidGoal, DetectCentroidAction
from behavior_manager.interfaces.manipulation_behavior_new import FullyExtendTorso, MoveTorsoBehavior, PickBehavior, TuckWithCondBehavior, PlaceBehavior, GrasplocPickBehavior, HeadMoveJointBehavior, HeadMoveBehavior, DustingBehavior
from behavior_manager.conditions.arm_tucked_condition import ArmTuckedCondition
from behavior_manager.interfaces.centroid_detector_behavior import CentroidDetectorBehavior
from behavior_manager.interfaces.tts_behavior import TTSBehavior, TTSFormatedBehavior
from behavior_manager.interfaces.update_joints_behavior import JointToBlackboardBehavior
from behavior_manager.interfaces.navigation_behavior import *
from behavior_manager.conditions.at_pose_condition import AtPoseCondition
from behavior_manager.interfaces.detect_handles_behavior import DetectHandlesBehavior
from behavior_manager.interfaces.grab_bag_behavior import GrabBagBehavior
from behavior_manager.interfaces.grasploc_behavior import GrasplocBehavior
from behavior_manager.interfaces.sleep_behavior import SleepBehavior
from behavior_manager.interfaces.object_detector_behavior import ObjectDetectorBehavior
# TODO(Someone): Figure out how to actually import the module
from lfd_behaviors import GetFeatherDusterOrientation, GetMFCDusterOrientation
from behavior_manager.interfaces.fetch_manipulation_behavior import ControlGripperBehavior, LoadAndExecuteTrajectoryBehavior, AddBoxBehavior, RemoveCollisionObjBehavior

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
    seq = py_trees.composites.Sequence(name)
    seq.add_children([
      TuckWithCondBehavior(name, 'tuck'),
      PickBehavior(name, centroid_bb_key='detect_centroid')])
    return seq
def BuildCentroidDetectorBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(name + '/min_x', 0.0)
    blackboard.set(name + '/max_x', 1.5)
    blackboard.set(name + '/min_y', -0.5)
    blackboard.set(name + '/max_y', 0.5)
    blackboard.set(name + '/min_z', 0.65)
    blackboard.set(name + '/max_z', 1.00)
    return CentroidDetectorBehavior(name, centroid_bb_key=name)
def BuildPersonDetectorBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(name + '/min_x', 0.25)
    blackboard.set(name + '/max_x', 2.0)
    blackboard.set(name + '/min_y', -0.5)
    blackboard.set(name + '/max_y', 0.5)
    blackboard.set(name + '/min_z', 1.5)
    blackboard.set(name + '/max_z', 2.0)
    return CentroidDetectorBehavior(name, centroid_bb_key=name)
def BuildDetectHandHandBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(name + '/min_x', 0.0)
    blackboard.set(name + '/max_x', 1.0)
    blackboard.set(name + '/min_y', -0.5)
    blackboard.set(name + '/max_y', 0.5)
    blackboard.set(name + '/min_z', 0.80)
    blackboard.set(name + '/max_z', 1.12)
    return CentroidDetectorBehavior(name, centroid_bb_key=name)
def BuildTuckWithCondBehavior(name, tuck_pose='tuck'):
    return TuckWithCondBehavior(name, tuck_pose)
def BuildPlaceBehavior(name):
    return PlaceBehavior(name)
def BuildHeadMoveBehavior(name):
    return HeadMoveBehavior(name, None, 1, 0, 0.5)
def BuildLookAtPersonBehavior(name):
    return HeadMoveBehavior(name, None, 1, 0, 1.75)
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
def BuildNavBehavior(name, *args, **kwargs):
    return NavOverride(name + '_override', NavToPoseWithCondBehavior(name, *args, **kwargs))
def BuildAtPoseBehavior(name, *args, **kwargs):
    root = py_trees.composites.Selector(name='root_'+name)
    seq = py_trees.composites.Sequence(name='seq_'+name)
    cond_at_pose = AtPoseCondition(name, *args, **kwargs)
    act_write_true = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name=name, variable_value=True)
    act_write_false = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name=name, variable_value=False)

    root.add_children([seq, act_write_false])
    seq.add_children([cond_at_pose, act_write_true])
    return root
def BuildArmTuckedBehavior(name, tuck_pose='tuck'):
    root = py_trees.composites.Selector(name='root_'+name)
    seq = py_trees.composites.Sequence(name='seq_'+name)
    cond_at_pose = ArmTuckedCondition(name, tuck_pose)
    act_write_true = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name=name, variable_value=True)
    act_write_false = py_trees.blackboard.SetBlackboardVariable(name=name, variable_name=name, variable_value=False)

    root.add_children([seq, act_write_false])
    seq.add_children([cond_at_pose, act_write_true])
    return root
# def BuildBagDetectBehavior(name):
#     return DetectHandlesBehavior(name)
# def BuildBagGrabBehavior(name):
#     return GrabBagBehavior(name)
# def BuildGrasplocBehavior(name):
#     return GrasplocBehavior(name, 'centroid')
# def BuildGrasplocPickBehavior(name):
#     return GrasplocPickBehavior(name)
def BuildSleepBehavior(name, duration=1):
    return SleepBehavior(name, duration)
def BuildSetObjDetectorTarget(name, target):
    return py_trees.blackboard.SetBlackboardVariable(name=name, variable_name='item', variable_value=target)
def BuildPickObjBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(name + '/min_x', 0.0)
    blackboard.set(name + '/max_x', 1.0)
    blackboard.set(name + '/min_y', -0.5)
    blackboard.set(name + '/max_y', 0.5)
    blackboard.set(name + '/min_z', 0.62)
    blackboard.set(name + '/max_z', 1.12)

    sel = py_trees.composites.Selector(name)
    act_say_fail = TTSFormatedBehavior(name, 'I could not pick up the {0}', {0: 'item'})
    act_say_starting = TTSFormatedBehavior(name, 'I will pick up the {0}', {0: 'item'})
    para = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, synchronize=True, allow_failure=True)
    act_tuck_arm = TuckWithCondBehavior('act_tuck_arm', 'tuck')
    act_move_head_down = HeadMoveJointBehavior('head_move_joint_down', pan=0, tilt=0.906)
    act_detect_item = ObjectDetectorBehavior('act_detect_item', bounding_box_bb_key=name+'/bb')
    act_detect_centroid = CentroidDetectorBehavior(name, bounding_box_bb_key=name+'/bb')
    act_grasploc = GrasplocBehavior('grasploc', 'centroid')
    act_grasploc_pick = GrasplocPickBehavior(name='grasplocPick')# , filter_off=True)
    act_tuck3_arm = TuckWithCondBehavior(name, 'unknown_3')
    seq_root = py_trees.composites.Sequence(name=name)

    sel.add_children([seq_root, act_say_fail])
    seq_root.add_children([
      para,
      act_detect_item,
      act_detect_centroid,
      act_grasploc,
      act_grasploc_pick,
      act_tuck3_arm
    ])

    para.add_children([
      act_tuck_arm,
      act_move_head_down,
      act_say_starting])

    return sel
def BuildPickAnythingBehavior(name):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(name + '/min_x', 0.0)
    blackboard.set(name + '/max_x', 1.0)
    blackboard.set(name + '/min_y', -0.5)
    blackboard.set(name + '/max_y', 0.5)
    blackboard.set(name + '/min_z', 0.62)
    blackboard.set(name + '/max_z', 1.12)

    sel = py_trees.composites.Selector(name)
    act_say_fail = TTSFormatedBehavior(name, 'I could not pick up the {0}', {0: 'item'})
    act_say_starting = TTSFormatedBehavior(name, 'I will pick up the {0}', {0: 'item'})
    para = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, synchronize=True, allow_failure=True)
    act_tuck_arm = TuckWithCondBehavior('act_tuck_arm', 'tuck')
    act_move_head_down = HeadMoveJointBehavior('head_move_joint_down', pan=0, tilt=0.906)
    act_detect_centroid = CentroidDetectorBehavior(name)
    act_grasploc = GrasplocBehavior('grasploc', 'centroid')
    act_grasploc_pick = GrasplocPickBehavior(name='grasplocPick')# , filter_off=True)
    act_tuck3_arm = TuckWithCondBehavior(name, 'unknown_3')
    seq_root = py_trees.composites.Sequence(name=name)

    sel.add_children([seq_root, act_say_fail])
    seq_root.add_children([
      para,
      act_detect_centroid,
      act_grasploc,
      act_grasploc_pick,
      act_tuck3_arm
    ])

    para.add_children([
      act_tuck_arm,
      act_move_head_down,
      act_say_starting])

    return sel
def BuildOpenGripperBehavior(name):
    para = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, synchronize=True, allow_failure=True)
    tts = TTSBehavior(name, 'I can not use the duster in this orientation')
    drop = ControlGripperBehavior(name, 1.0)
    para.add_children([drop, tts])
    return para
def BuildCloseGripperBehavior(name):
    return ControlGripperBehavior(name, 0.0)
def BuildGetFeatherDusterOrientation(name):
    para = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, synchronize=True, allow_failure=True)
    tts = TTSBehavior(name, 'I am checking the orientation of the duster')
    seq = py_trees.composites.Sequence(name)
    seq.add_children([
      py_trees.blackboard.SetBlackboardVariable(name=name, variable_name='item', variable_value='feather'),
      ObjectDetectorBehavior('act_detect_item'),
      GetFeatherDusterOrientation(name),
    ])
    para.add_children([tts, seq])
    return para
def BuildGetMFCDusterOrientation(name):
    para = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, synchronize=True, allow_failure=True)
    tts = TTSBehavior(name, 'I am checking the orientation of the duster')
    check = GetMFCDusterOrientation(name)
    para.add_children([tts, check])
    return para
def BuildDustBehavior(name, duster_type):
    para = py_trees.composites.Parallel(name, policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL, synchronize=True, allow_failure=True)
    tts = TTSBehavior(name, 'Dusting is the best. I love to dust so much. I am a happy robot')
    dust = DustingBehavior(name, duster_type=duster_type)
    para.add_children([dust, tts])
    return para
def BuildCheckObjBehavior(name):
    seq = py_trees.composites.Sequence(name)
    act_look_at_duster = HeadMoveJointBehavior(name, pan=0, tilt=0.195)
    seq.add_children([
      act_look_at_duster,
      LoadAndExecuteTrajectoryBehavior(name, traj_name='LfdCheckObj')])
    return seq
def BuildPlaceDusterBehavior(name):
    seq = py_trees.composites.Sequence(name)
    seq.add_children([
      LoadAndExecuteTrajectoryBehavior(name, traj_name='LfdPlaceWrongItem'),
      ControlGripperBehavior(name, 1.0)])
    return seq
