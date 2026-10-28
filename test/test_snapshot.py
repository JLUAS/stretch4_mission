import json
import subprocess
import sys
from pathlib import Path

import pytest

from stretch4_mission import snapshot
from stretch4_mission.mission_ui import MissionUI
from stretch4_mission.policy_bridge import PolicyBridge


class UI:
    def __init__(self, log_dir):
        self.log_dir = log_dir


def test_pictures_are_served_only_from_the_snapshots_folder(tmp_path):
    run = tmp_path / 'snapshots' / 'policy_20261028-140250'
    run.mkdir(parents=True)
    (run / 'gripper.jpg').write_bytes(b'jpeg')
    (tmp_path / 'secret.jpg').write_bytes(b'no')
    ui = UI(tmp_path)
    assert MissionUI.snapshot(ui, 'policy_20261028-140250', 'gripper') == b'jpeg'
    for folder, camera in (('..', 'secret'), ('policy_20261028-140250', '../../secret'), ('nope', 'gripper'),
                           ('policy_20261028-140250', 'head_right')):   # missing picture
        with pytest.raises(ValueError):
            MissionUI.snapshot(ui, folder, camera)


def test_snapshot_without_the_recorder_reports_why(tmp_path):
    proc = subprocess.run([sys.executable, str(Path(snapshot.__file__)), '--out', str(tmp_path / 'x')],
                          capture_output=True, text=True)
    assert proc.returncode == 1 and json.loads(proc.stdout.splitlines()[-1])['error']


class Proc:
    def __init__(self, alive):
        self.alive = alive

    def poll(self):
        return None if self.alive else 0


class Bridge:
    snapshot_cameras = ['head_right', 'gripper']

    def __init__(self, procs, started_at):
        self.procs, self.started_at, self.photos = procs, started_at, None


@pytest.mark.parametrize('procs, started_at', [({}, 1.0),                              # lerobot: no stream
                                               ({'stream': Proc(False)}, 1.0),         # stream already gone
                                               ({'stream': Proc(True)}, None)])        # the policy never started
def test_no_picture_when_there_is_nothing_to_take_it_from(procs, started_at):
    bridge = Bridge(procs, started_at)
    PolicyBridge._snapshot(bridge)   # returns before running anything
    assert bridge.photos is None
