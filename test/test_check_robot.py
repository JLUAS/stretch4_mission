from stretch4_mission.check_robot import FAIL, OK, WARN, check_policies


def episode(root, name, text):
    (root / name).mkdir(parents=True)
    (root / name / 'records.jsonl').write_text(text)


def run(gen):
    results = []
    try:
        while True:
            results.append(next(gen))
    except StopIteration as stop:
        return results, stop.value


def test_check_policies_reports_cut_off_and_missing_data(tmp_path):
    good, cut, gone = tmp_path / 'good', tmp_path / 'cut', tmp_path / 'gone'
    episode(good, 'episode_a', '{"t": 0}\n{"t": 20}\n')
    episode(good, 'episode_b', '{"t": 0}\n{"t": 5, "x": "cut o')        # cut off: does not count
    (good / 'start_pose.json').write_text('{}')
    episode(cut, 'episode_a', '{"t": 0}\n{"t": 9, "x": "cut o')         # only cut-off episodes
    (cut / 'start_pose.json').write_text('{}')
    f = tmp_path / 'p.yaml'
    f.write_text('runtime_margin_sec: 10\npolicies:\n'
                 + ''.join(f'  {n}:\n    root: {tmp_path / n}\n    ckpt: ~/c\n    max_runtime_sec: auto\n'
                           for n in ('good', 'cut', 'gone')))
    results, policies = run(check_policies(f))
    by_text = {text.split(':')[0]: (level, text) for level, text, _ in results}
    assert set(policies) == {'good', 'cut', 'gone'}
    assert by_text['good'][0] == WARN and '30.0 s' in by_text['good'][1] and '1/2 episodes' in by_text['good'][1]
    assert by_text['cut'][0] == FAIL and 'cut off' in by_text['cut'][1]
    assert by_text['gone'][0] == FAIL and 'missing' in by_text['gone'][1]
    assert results[0][0] == OK
