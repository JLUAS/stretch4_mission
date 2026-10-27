import pytest

from stretch4_mission.mission_controller import MissionController


class Controller:
    p = {
        'lift_joint': 'lift_joint',
        'lift_retract_position': 0.5,
        'lift_retract_tolerance': 0.01,
        'lift_retract_duration': 6.0,
        'lift_timeout': 30.0,
        'arm_joint': 'arm_joint',
        'arm_state_joints': ['arm_l1_joint', 'arm_l2_joint'],
        'arm_retract_duration': 5.0,
        'arm_timeout': 30.0,
        'arm_retracted_max': 0.02,
        'wrist_yaw_joint': 'wrist_yaw_joint',
        'wrist_roll_joint': 'wrist_roll_joint',
        'wrist_pitch_joint': 'wrist_pitch_joint',
        'wrist_yaw_stow_position': 3.14,
        'wrist_roll_stow_position': 0.0,
        'wrist_pitch_stow_position': 0.0,
        'wrist_stow_tolerance': 0.03,
        'wrist_stow_duration': 4.0,
    }

    def __init__(self, lift):
        self.joints = {'lift_joint': lift, 'arm_l1_joint': 0.1, 'arm_l2_joint': 0.1,
                       'wrist_yaw_joint': 0.0, 'wrist_roll_joint': 0.0, 'wrist_pitch_joint': 0.5}
        self.moves = []

    def move_joint(self, what, joint, target, duration, timeout, measure, ok):
        self.moves.append((what, joint, target))
        if joint == 'arm_joint':
            self.joints.update(arm_l1_joint=0.0, arm_l2_joint=0.0)
        else:
            self.joints[joint] = target
        assert ok(measure())


WRIST = [('Stow wrist yaw', 'wrist_yaw_joint', 3.14), ('Stow wrist roll', 'wrist_roll_joint', 0.0),
         ('Stow wrist pitch', 'wrist_pitch_joint', 0.0)]
LIFT, RETRACT = ('Lift to travel height', 'lift_joint', 0.5), ('Retract arm', 'arm_joint', 0.0)


@pytest.mark.parametrize('lift, first_two', [(0.2, [LIFT, RETRACT]),     # below 0.5 m: lift, then retract
                                             (0.9, [RETRACT, LIFT]),     # above: retract, then lower
                                             (0.5, [RETRACT, LIFT])])    # at travel height: retract first
def test_order_depends_on_lift_height_and_wrist_is_last(lift, first_two):
    controller = Controller(lift)
    MissionController.retract_arm(controller)
    assert controller.moves == first_two + WRIST
    assert all('gripper' not in joint for _, joint, _ in controller.moves)


def test_no_lift_reading_moves_nothing():
    controller = Controller(None)
    with pytest.raises(Exception, match='No lift_joint position'):
        MissionController.retract_arm(controller)
    assert controller.moves == []
