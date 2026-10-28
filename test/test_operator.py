import logging
import time

from stretch4_mission.mission_ui import COMPONENTS, MissionUI


class Mission:
    def __init__(self):
        self.on = False

    def running(self):
        return self.on


class Robot:
    """What MissionUI._operate uses, with every part off and the robot not localized."""
    operator_map, operate, operator_stop = MissionUI.operator_map, MissionUI.operate, MissionUI.operator_stop

    def __init__(self, plan_mission='ask_1', mission_state='COMPLETED'):
        self.up, self.started, self.events = set(), [], []
        self.is_localized, self.localizing, self.localize_text = False, False, 'NOT localized'
        self.plan_mission, self.mission_state = plan_mission, mission_state
        self.mission, self.trace, self.op_cancel = Mission(), None, False
        self.op = {'state': 'idle', 'request': '', 'message': '', 'steps': [], 'mission': None}

    def get_logger(self):
        return logging.getLogger('test')

    def healthy(self, name):
        return name in self.up

    def current_map(self):
        return '/maps/lab.yaml' if 'nav2' in self.up else None

    def component_status(self, name):
        return {'startable': name not in self.started}

    def start_component(self, name, map_path):
        self.started.append(name)
        self.up.add(name)   # ready at once

    def plan(self, text, nav_map):
        self.events.append(('plan', nav_map))
        return {'message': 'I will clean the table.', 'mission': self.plan_mission, 'plan': ['config:table'],
                'tasks': [{'title': 'Clean the table', 'where': 'dining table'}]}

    def localized(self):
        return self.is_localized

    def localize(self):
        self.events.append(('localize',))
        self.is_localized = True

    def start_mission(self, name):
        self.events.append(('start', name))
        self.trace = {'mission': name, 'state': self.mission_state, 'detail': 'Navigation ended with FAILED'}

    def configs(self):
        return {'a': {'map': '/maps/lab.yaml'}, 'b': {'map': '/maps/home.yaml'}, 'c': {'map': '/maps/home.yaml'}}, {}

    def missions(self):
        return [{'name': 'm', 'map': '/maps/lab.yaml', 'steps': []}]

    default_map = '/maps/lab.yaml'


def test_everything_off_is_started_in_order_then_localized_then_run():
    robot = Robot()
    MissionUI._operate(robot, 'clean the table')
    assert robot.started == [name for name, _, _ in COMPONENTS]   # driver, lidars, Nav2, bridge
    assert robot.events == [('plan', '/maps/lab.yaml'), ('localize',), ('start', 'ask_1')]   # plan before anything moves
    assert robot.op['state'] == 'done' and robot.op['message'] == 'I will clean the table.'
    assert [s['status'] for s in robot.op['steps']] == ['done'] * len(robot.op['steps'])
    assert robot.op['steps'][-1]['text'] == 'Doing the task'


def test_a_request_without_a_plan_starts_nothing():
    robot = Robot(plan_mission=None)
    MissionUI._operate(robot, 'make me a coffee')
    assert robot.op['state'] == 'cannot' and robot.started == [] and robot.events == [('plan', '/maps/lab.yaml')]


def test_a_failed_mission_is_reported_with_its_reason():
    robot = Robot(mission_state='FAILED')
    MissionUI._operate(robot, 'clean the table')
    assert robot.op['state'] == 'failed' and robot.op['error'] == 'Navigation ended with FAILED'
    assert robot.op['steps'][-1]['status'] == 'failed'


def test_a_part_that_never_gets_ready_fails_the_request():
    robot = Robot()
    robot.start_component = lambda name, map_path: robot.started.append(name)   # starts, never answers
    MissionUI._operate(robot, 'clean the table', component_timeout=0.6)
    assert robot.op['state'] == 'failed' and 'did not get ready' in robot.op['error'] and robot.started == ['driver']


def test_with_nav2_off_the_map_with_most_setup_is_used():
    robot = Robot()
    assert MissionUI.operator_map(robot) == '/maps/lab.yaml'   # lab: 1 config + 1 mission; home: 2 configs; tie -> default
    robot.default_map = '/maps/other.yaml'
    assert MissionUI.operator_map(robot) in ('/maps/lab.yaml', '/maps/home.yaml')
    robot.up.add('nav2')
    assert MissionUI.operator_map(robot) == '/maps/lab.yaml'   # Nav2's own map wins


def test_stop_ends_the_request_before_the_next_step():
    robot = Robot()
    robot.localize = lambda: (setattr(robot, 'localizing', True), setattr(robot, 'op_cancel', True))   # Stop while turning
    robot.localizing, robot.spin_goal, robot.localize_cancel = False, None, False
    start = time.monotonic()
    MissionUI._operate(robot, 'clean the table')
    assert robot.op['state'] == 'stopped' and ('start', 'ask_1') not in robot.events and time.monotonic() - start < 5


def test_steps_and_suggestions_are_in_words_from_the_setup():
    from stretch4_mission.planner import describe_step, suggestions
    policies = {'clean_table': {'task': 'Clean the table'}, 'grab_sponge': {'task': ''}}
    configs = {'table': {'policy': 'clean_table', 'label': 'dining table', 'map': '/m.yaml'},
               'shelf': {'policy': 'grab_sponge', 'map': '/m.yaml'}}
    assert describe_step({'type': 'policy', 'policy': 'clean_table', 'config': 'table'}, policies, configs) == \
        {'title': 'Clean the table', 'where': 'dining table'}
    assert describe_step({'type': 'policy', 'policy': 'grab_sponge', 'config': 'shelf'}, policies, configs) == \
        {'title': 'grab sponge', 'where': 'shelf'}
    assert describe_step({'type': 'navigate', 'label': 'kitchen'}, policies, configs)['title'] == 'Going to kitchen'
    assert describe_step({'type': 'command', 'command': 'x'}, policies, configs)['title'] == 'Running a command'
    missions = [{'name': 'tour_home'}, {'name': 'ask_20261028_1'}]
    assert suggestions(policies, configs, missions) == ['Clean the table', 'grab sponge',
                                                        'Run the tour home mission']   # text-request missions left out
    configs['table2'] = {'policy': 'clean_table', 'label': 'kitchen table', 'map': '/m.yaml'}   # same task, two places
    assert suggestions(policies, configs, [])[:3:2] == ['Clean the table at the dining table',
                                                        'Clean the table at the kitchen table']
