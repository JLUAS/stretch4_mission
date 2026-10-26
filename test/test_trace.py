from stretch4_mission.mission_controller import MissionTrace

STEPS = [{'policy': 'a', 'goal': {}}, {'policy': 'b', 'goal': {}}]


def phases(t, i):
    return t.data['steps'][i]['phases']


def test_trace_walks_steps_and_fails_the_active_phase():
    t = MissionTrace('m', STEPS)
    t.enter('INITIALIZING')
    t.enter('RETRACTING_ARM', 0)
    t.enter('CHECKING_POSE', 0)
    t.skip(0, 'NAVIGATING')
    t.enter('ALIGNING', 0)
    t.enter('PREPARING_POLICY', 0)
    t.enter('EXECUTING_POLICY', 0)
    t.enter('RETRACTING_ARM', 1)
    assert phases(t, 0) == {'RETRACTING_ARM': 'done', 'CHECKING_POSE': 'done', 'NAVIGATING': 'skipped',
                            'ALIGNING': 'done', 'PREPARING_POLICY': 'done', 'EXECUTING_POLICY': 'done'}
    t.enter('NAVIGATING', 1)
    t.fail('Navigation ended with FAILED')
    assert phases(t, 1)['NAVIGATING'] == 'failed' and phases(t, 1)['PREPARING_POLICY'] == 'pending'
    assert t.data['state'] == 'FAILED' and t.data['step'] == 1


def test_trace_complete():
    t = MissionTrace('m', STEPS[:1])
    t.enter('EXECUTING_POLICY', 0)
    t.complete('done')
    assert phases(t, 0)['EXECUTING_POLICY'] == 'done' and t.data['state'] == 'COMPLETED'
