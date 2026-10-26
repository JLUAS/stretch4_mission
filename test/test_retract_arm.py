from stretch4_mission.mission_controller import MissionController


def test_lift_then_arm_then_wrist_stow_without_gripper():
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
        joints = {
            'lift_joint': 0.5,
            'arm_l1_joint': 0.005,
            'arm_l2_joint': 0.005,
            'wrist_yaw_joint': 3.14,
            'wrist_roll_joint': 0.0,
            'wrist_pitch_joint': 0.0,
        }

        def __init__(self):
            self.moves = []

        def move_joint(self, what, joint, target, duration, timeout, measure, ok):
            self.moves.append((what, joint, target))
            assert ok(measure())

    controller = Controller()
    MissionController.retract_arm(controller)
    assert controller.moves == [
        ('Lift before retract', 'lift_joint', 0.5),
        ('Retract arm', 'arm_joint', 0.0),
        ('Stow wrist yaw', 'wrist_yaw_joint', 3.14),
        ('Stow wrist roll', 'wrist_roll_joint', 0.0),
        ('Stow wrist pitch', 'wrist_pitch_joint', 0.0),
    ]
    assert all('gripper' not in joint for _, joint, _ in controller.moves)
