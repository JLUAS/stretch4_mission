import os
import time

import pytest

from stretch4_mission import mission_controller
from stretch4_mission.mission_controller import MissionController, MissionFailed


class Controller:
    p = {'command_timeout': 5.0}
    run_command, stop_command = MissionController.run_command, MissionController.stop_command

    def __init__(self):
        self.command, self.states, self.errors = None, [], []

    def set_state(self, state, step=None, detail=''):
        self.states.append(state)

    def spin_for(self, seconds):
        time.sleep(0.05)

    def info(self, msg):
        pass

    def error(self, msg):
        self.errors.append(msg)


@pytest.fixture(autouse=True)
def logs(tmp_path, monkeypatch):
    monkeypatch.setattr(mission_controller, 'LOG_DIR', tmp_path)
    return tmp_path


def test_exit_zero_passes_and_output_is_logged(logs):
    c = Controller()
    result = c.run_command(0, {'type': 'command', 'command': 'echo hello from $PWD'})
    assert result.startswith('exit code 0') and c.states == ['RUNNING_COMMAND'] and c.command is None
    assert 'hello from' in next(logs.glob('command_*.log')).read_text()


def test_nonzero_exit_fails_the_mission():
    with pytest.raises(MissionFailed, match='exited with code 3'):
        Controller().run_command(0, {'type': 'command', 'command': 'exit 3'})


def test_timeout_stops_the_whole_process_group(logs):
    c = Controller()
    pid_file = logs / 'child.pid'
    with pytest.raises(MissionFailed, match='still running after 0 s'):
        c.run_command(0, {'type': 'command', 'command': f'sleep 30 & echo $! > {pid_file}; wait', 'timeout_sec': 0.5})
    assert c.command is None and not c.errors
    child = int(pid_file.read_text())
    time.sleep(0.2)
    with pytest.raises(ProcessLookupError):   # the script's own child is gone too
        os.kill(child, 0)
