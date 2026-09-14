"""Host runner behavior, with a file-only child in place of ROS/Go2."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

import run_saved_robot_goal as runner
from test_robot_goal_campaign import collected, fake_prepare


@pytest.fixture
def saved_plan(collected):
    runner.campaign.bind_goals(collected)
    return runner.read(collected / 'goal-plan.json')


def test_all_labels_reuse_unused_trials_but_preserve_used_attempts(collected, monkeypatch):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    first = [runner.prepared_trial(collected, str(i)) for i in range(1, 6)]
    assert [x['goal_label'] for x in first] == ['1', '2', '3', '4', '5']
    assert runner.prepared_trial(collected, '3')['path'] == first[2]['path']
    run = Path(first[2]['path'])
    (run / 'USED').mkdir()
    (run / 'evidence.bin').write_bytes(b'original data')
    retry = runner.prepared_trial(collected, '3')
    assert retry['trial_id'] == 'full-goal-3-002'
    assert (run / 'evidence.bin').read_bytes() == b'original data'


def test_changed_goal_or_map_rejected(collected, monkeypatch):
    runner.campaign.bind_goals(collected)
    (collected / 'mapping/map.db').write_bytes(b'changed')
    with pytest.raises(ValueError, match='지도'):
        runner.prepared_trial(collected, '1')
    (collected / 'goal-plan.json').write_text('{}')
    with pytest.raises(ValueError, match='목표 파일'):
        runner.load_campaign(collected)


def test_full_and_direct_keep_distinct_attempts_with_identical_targets(collected, monkeypatch):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    for label in map(str, range(1, 6)):
        full = runner.prepared_trial(collected, label)
        direct = runner.prepared_trial(collected, label, 'direct_goal')
        assert full['method'] == 'full' and direct['method'] == 'direct_goal'
        assert full['path'] != direct['path']
        assert (Path(full['path'])/'goal-plan.json').read_bytes() == (Path(direct['path'])/'goal-plan.json').read_bytes()
        assert runner.prepared_trial(collected, label)['path'] == full['path']
        assert runner.prepared_trial(collected, label, 'direct_goal')['path'] == direct['path']
    old = Path(direct['path'])
    (old/'USED').mkdir()
    assert runner.prepared_trial(collected, '5', 'direct_goal')['trial_id'] == 'direct_goal-goal-5-002'
    assert runner.prepared_trial(collected, '5')['path'] == full['path']


@pytest.mark.parametrize('wrong', ['binding', 'manifest'])
def test_direct_rejects_trial_with_full_runtime_or_binding(collected, monkeypatch, wrong):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    old = runner.prepared_trial(collected, '2', 'direct_goal')
    p = Path(old['path'])/('campaign-binding.json' if wrong == 'binding' else 'manifest.json')
    d = runner.read(p)
    if wrong == 'binding':
        d['method'] = 'full'
    else:
        d['runtime_conditions']['navigation'] = 'full'
    runner.save(p, d)
    assert runner.prepared_trial(collected, '2', 'direct_goal')['path'] != old['path']
    assert p.exists()


def test_direct_check_prepares_all_labels_without_camera_or_ros(collected, tmp_path, monkeypatch):
    runner.campaign.bind_goals(collected)
    monkeypatch.setattr(runner.campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(runner.campaign, 'validate_trial', lambda *args: None)
    monkeypatch.setattr(runner, 'STATE', tmp_path/'state')
    monkeypatch.setattr(sys, 'argv', ['runner', '--root', str(collected), '--check', '--navigation-mode', 'direct_goal'])
    def unexpected(*args, **kwargs):
        raise AssertionError('File-only check attempted to start a process')
    monkeypatch.setattr(subprocess, 'run', unexpected)
    monkeypatch.setattr(subprocess, 'Popen', unexpected)
    assert runner.main() == 0
    entries = runner.read(collected/'campaign.json')['trials']
    assert len(entries) == 5 and all(e['method'] == 'direct_goal' for e in entries)


@pytest.mark.parametrize('method', ['full', 'direct_goal'])
def test_pending_replacement_blocks_old_target_and_preserves_other_goals(collected, monkeypatch, method):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    old = runner.prepared_trial(collected, '5', method)
    first = runner.prepared_trial(collected, '1', method)
    original = (collected/'goal-plan.json').read_bytes()
    c = runner.read(collected/'campaign.json')
    c['pending_goal_replacements'] = {'5': {'status': 'pending_capture'}}
    runner.save(collected/'campaign.json', c)
    with pytest.raises(ValueError, match='새 목표 좌표 등록 대기'):
        runner.prepared_trial(collected, '5', method)
    assert runner.prepared_trial(collected, '1', method)['path'] == first['path']
    assert Path(old['path']).exists() and not (Path(old['path'])/'USED').exists()
    assert (collected/'goal-plan.json').read_bytes() == original


@pytest.mark.parametrize('option', ['--list', '--check'])
def test_pending_goal_is_announced_without_being_prepared(collected, tmp_path, monkeypatch, capsys, option):
    runner.campaign.bind_goals(collected)
    monkeypatch.setattr(runner.campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(runner.campaign, 'validate_trial', lambda *args: None)
    monkeypatch.setattr(runner, 'STATE', tmp_path/'state')
    c = runner.read(collected/'campaign.json')
    c['pending_goal_replacements'] = {'5': {'status': 'pending_capture'}}
    runner.save(collected/'campaign.json', c)
    monkeypatch.setattr(sys, 'argv', ['runner', '--root', str(collected), option, '--navigation-mode', 'direct_goal'])
    assert runner.main() == (2 if option == '--check' else 0)
    output = capsys.readouterr().out
    assert '5번: 새 목표 좌표 등록 대기' in output and '5번: x=' not in output
    entries = runner.read(collected/'campaign.json')['trials']
    assert all(e['goal_label'] != '5' for e in entries)


@pytest.mark.parametrize('speed', [.6, .8])
def test_campaign_speed_revision_creates_new_trial_for_each_goal(collected, monkeypatch, speed):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    old = [runner.prepared_trial(collected, str(i)) for i in range(1,6)]
    settings = campaign.read(collected / 'campaign.json')
    settings.update(trial_settings={'look':'forward_0p2','turn_speed_radps':speed},
                    runtime_revision='look20-turn-speed-revision')
    campaign.write(collected / 'campaign.json', settings)
    for prior in old:
        new = runner.prepared_trial(collected, prior['goal_label'])
        assert new['path'] != prior['path']
        condition = campaign.read(Path(new['path']) / 'manifest.json')['runtime_conditions']
        assert condition['look_execution'] == 'forward_0p2'
        assert condition['turn_speed_radps'] == speed
        assert runner.prepared_trial(collected, prior['goal_label'])['path'] == new['path']
        assert not (Path(prior['path']) / 'USED').exists()


@pytest.mark.parametrize('method', ['full', 'direct_goal'])
def test_longer_episode_replaces_prepared_attempt_without_rewriting_old_evidence(collected, monkeypatch, method):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    old = runner.prepared_trial(collected, '4', method)
    old_manifest = Path(old['path'])/'manifest.json'
    original = old_manifest.read_bytes()
    c = campaign.read(collected/'campaign.json')
    c['trial_settings'] = {'max_duration_s':1800}
    campaign.write(collected/'campaign.json', c)
    new = runner.prepared_trial(collected, '4', method)
    assert new['path'] != old['path']
    assert old_manifest.read_bytes() == original
    assert runner.read(Path(new['path'])/'manifest.json')['runtime_conditions']['max_duration_s'] == 1800
    assert runner.runner_timeout_s(Path(new['path'])) == 1980
    assert runner.prepared_trial(collected, '4', method)['path'] == new['path']


@pytest.mark.parametrize('condition', ['cancel', 'stop', 'expired', 'valid'])
def test_dispatch_requires_current_readiness_and_respects_stop(tmp_path, condition, saved_plan):
    plan = saved_plan
    if condition == 'cancel':
        (tmp_path / 'CANCEL').touch()
    if condition == 'stop':
        (tmp_path / 'full5m/runtime').mkdir(parents=True)
        (tmp_path / 'full5m/runtime/STOP').touch()
    confirmed = time.time() - (301 if condition == 'expired' else 1)
    if condition == 'expired':
        with pytest.raises(ValueError):
            runner.dispatch(tmp_path, plan, confirmed)
    else:
        assert runner.dispatch(tmp_path, plan, confirmed) is (condition == 'valid')
    assert (tmp_path / 'START_GOAL').exists() is (condition == 'valid')
    if condition == 'valid':
        assert runner.read(tmp_path / 'START_GOAL')['confirmed_unix'] == confirmed


FAKE_CHILD = '''from pathlib import Path
import json,sys,time
r=Path(sys.argv[1]);rt=r/'full5m/runtime';rt.mkdir(parents=True)
(r/'STACK_STARTED').touch()
end=time.monotonic()+5
while not (r/'START_GOAL').exists() and not (r/'CANCEL').exists():
 if time.monotonic()>end:raise SystemExit(9)
 time.sleep(.01)
if (r/'CANCEL').exists():raise SystemExit(1)
(rt/'goal.json').write_text('{}')
if len(sys.argv)>2:
 (rt/'STOP').touch();(r/'CANCEL').touch()
 time.sleep(.5)
(rt/'result.json').write_text(json.dumps({'goal_published':True,'reason':'FAKE_ONLY','shutdown_service_confirmed':True}))
(r/'RECORDERS_CLOSED').touch()
'''


@pytest.mark.parametrize('stop', [False, True])
def test_file_only_child_handshake_and_stop_cleanup(tmp_path, monkeypatch, stop, saved_plan):
    import shlex
    state = tmp_path / 'state';state.mkdir()
    run = tmp_path / 'run';run.mkdir()
    child = tmp_path / 'fake.py';child.write_text(FAKE_CHILD)
    command = [sys.executable, str(child), str(run)] + (['stop'] if stop else [])
    (run / 'run_trial.sh').write_text('exec ' + ' '.join(map(shlex.quote, command)) + '\n')
    monkeypatch.setattr(runner, 'STATE', state)
    monkeypatch.setattr(runner, 'active_services', lambda: set())
    monkeypatch.setattr(runner, 'recording_ready', lambda run: True)
    before = signal.getsignal(signal.SIGINT)
    assert runner.execute(run, saved_plan, time.time()) == 0
    assert signal.getsignal(signal.SIGINT) == before
    assert not (state / 'active.json').exists()
    assert (run / 'RECORDERS_CLOSED').exists()
    assert (run / 'operator-stop.json').exists() is stop


def test_stop_before_child_launch_never_dispatches(tmp_path, monkeypatch):
    runner.stop_request(tmp_path, 'external_stop_command')
    assert not runner.dispatch(tmp_path, {'origin': {'origin_id': 'saved-origin'}}, time.time())
    assert not runner.read(tmp_path / 'operator-stop.json')['goal_was_published']


def test_user_stop_is_intervention_without_inventing_contact(tmp_path, monkeypatch):
    runner.save(tmp_path / 'field-report.json', {'status':'awaiting_field_report',
        'direct_intervention':None,'contact':None,'obstruction':None})
    rt = tmp_path / 'full5m/runtime';rt.mkdir(parents=True)
    (rt / 'goal.json').write_text('{}')
    runner.stop_request(tmp_path, 'signal_SIGINT')
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    runner.field_report(tmp_path)
    field = runner.read(tmp_path / 'field-report.json')
    assert field['direct_intervention'] is True
    assert field['contact'] is None and field['obstruction'] is None


def test_stop_after_finish_is_not_navigation_intervention(tmp_path, monkeypatch):
    runner.save(tmp_path / 'field-report.json', {'direct_intervention':None})
    rt = tmp_path / 'full5m/runtime';rt.mkdir(parents=True)
    (rt / 'goal.json').write_text('{}');(rt / 'result.json').write_text('{}')
    runner.stop_request(tmp_path, 'signal_SIGINT')
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    runner.field_report(tmp_path)
    assert runner.read(tmp_path / 'field-report.json')['direct_intervention'] is None


def test_recorder_failure_keeps_raw_archive_and_reports_incomplete(collected, monkeypatch):
    campaign = runner.campaign
    campaign.bind_goals(collected)
    monkeypatch.setattr(campaign, 'prepare_trial', fake_prepare)
    monkeypatch.setattr(campaign, 'validate_trial', lambda *args: None)
    monkeypatch.setattr(runner, 'active_services', lambda: set())
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    entry = runner.prepared_trial(collected, '4')
    run = Path(entry['path']);rt = run / 'full5m/runtime';rt.mkdir(parents=True)
    runner.save(rt / 'result.json', dict(goal_published=False, reason='PRESTART_FAILURE'))
    (rt / 'events.jsonl').write_text('')
    (run / 'RECORDERS_CLOSED').touch()
    assert runner.finish(collected, entry, 1) == 3
    assert (run / 'full5m/runtime/result.json').exists()
    assert (collected / 'archives' / entry['trial_id'] / 'raw-manifest.json').exists()
    summary = json.loads((collected / 'operator-results.jsonl').read_text())
    assert summary['recording_complete'] is False and summary['started'] is False
    assert summary['success'] is None


def test_global_lock_blocks_duplicate_runner(collected, tmp_path, monkeypatch):
    import fcntl
    runner.campaign.bind_goals(collected)
    state = tmp_path / 'state';state.mkdir()
    monkeypatch.setattr(runner, 'STATE', state)
    monkeypatch.setattr(sys, 'argv', ['runner', '--root', str(collected), '--check'])
    with (state / 'runner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='이미 실행'):
            runner.main()


def test_running_controller_blocks_new_drive(collected, tmp_path, monkeypatch):
    runner.campaign.bind_goals(collected)
    monkeypatch.setattr(runner, 'STATE', tmp_path / 'state')
    monkeypatch.setattr(sys, 'argv', ['runner', '2', '--root', str(collected)])
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr(runner, 'active_services', lambda: {'escape'})
    with pytest.raises(RuntimeError, match='기존 주행'):
        runner.main()


def test_signal_stops_child_before_finalization(tmp_path, monkeypatch, saved_plan):
    import shlex
    state = tmp_path / 'state';state.mkdir()
    run = tmp_path / 'run';run.mkdir()
    child = tmp_path / 'fake-signal.py'
    child.write_text(FAKE_CHILD.replace(
        "if len(sys.argv)>2:",
        "import os,signal\nos.kill(os.getppid(),signal.SIGINT)\n"
        "while not (rt/'STOP').exists():time.sleep(.01)\nif len(sys.argv)>2:"))
    (run / 'run_trial.sh').write_text('exec ' + ' '.join(map(shlex.quote,
        [sys.executable, str(child), str(run)])) + '\n')
    monkeypatch.setattr(runner, 'STATE', state)
    monkeypatch.setattr(runner, 'active_services', lambda: set())
    monkeypatch.setattr(runner, 'recording_ready', lambda run: True)
    assert runner.execute(run, saved_plan, time.time()) == 0
    request = runner.read(run / 'operator-stop.json')
    assert request['source'] == 'signal_SIGINT' and request['before_final_result']
    assert (run / 'RECORDERS_CLOSED').exists()


def test_cold_recorders_wait_until_two_passes_before_dispatch(tmp_path, monkeypatch, saved_plan):
    import shlex
    state = tmp_path / 'state';state.mkdir()
    run = tmp_path / 'run';run.mkdir()
    child = tmp_path / 'delayed.py';child.write_text(FAKE_CHILD.replace('+5', '+15'))
    (run / 'run_trial.sh').write_text('exec ' + ' '.join(map(shlex.quote,
        [sys.executable, str(child), str(run)])) + '\n')
    monkeypatch.setattr(runner, 'STATE', state)
    monkeypatch.setattr(runner, 'active_services', lambda: set())
    probes = []
    def readiness(run):
        assert not (run / 'START_GOAL').exists()
        probes.append(True)
        return len(probes) >= 3
    monkeypatch.setattr(runner, 'recording_ready', readiness)
    assert runner.execute(run, saved_plan, time.time()) == 0
    assert len(probes) == 4
    assert (run / 'START_GOAL').exists()


def test_startup_validation_never_dispatches(tmp_path, monkeypatch, saved_plan):
    import shlex
    state = tmp_path / 'state';state.mkdir()
    run = tmp_path / 'run';run.mkdir()
    child = tmp_path / 'verify.py';child.write_text(FAKE_CHILD.replace('+5', '+15'))
    (run / 'run_trial.sh').write_text('exec ' + ' '.join(map(shlex.quote,
        [sys.executable, str(child), str(run)])) + '\n')
    monkeypatch.setattr(runner, 'STATE', state)
    monkeypatch.setattr(runner, 'active_services', lambda: set())
    monkeypatch.setattr(runner, 'recording_ready', lambda run: True)
    assert runner.execute(run, saved_plan, time.time(), start_goal=False) == 1
    assert not (run / 'START_GOAL').exists()
    assert not (run / 'full5m/runtime/goal.json').exists()
    assert runner.read(run / 'no-motion-startup-validation.json')['recording_ready']
