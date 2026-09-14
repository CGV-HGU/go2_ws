"""Cross-check episode, shell and supervisor limits without ROS or motion."""
import pytest
from prepare_robot_pointgoal_trial import prepare
from validate_robot_pointgoal_trial import validate_timing, validate_recorder_timing
from run_saved_robot_goal import runner_timeout_s, required_recording_space_bytes, save


@pytest.mark.parametrize('duration', [180,360,1800])
def test_three_timeout_layers_agree(tmp_path, duration):
    wrapper = ('timeout --signal=TERM --kill-after=10s '+str(duration+60)+
               's docker compose exec --max-duration-s '+str(duration))
    settings = {'max_duration_s':duration}
    validate_timing(settings,wrapper,settings)
    save(tmp_path/'manifest.json',{'runtime_conditions':settings})
    assert runner_timeout_s(tmp_path)==duration+180


def test_long_episode_storage_budget_covers_observed_full_recording_rate():
    assert required_recording_space_bytes(360)==10*1024**3
    assert required_recording_space_bytes(1800)>=1800*28_000_000+10*1024**3


@pytest.mark.parametrize('duration', [0,-1,1801,float('nan'),float('inf'),True])
def test_invalid_duration_cannot_prepare_artifacts_or_start_runner(tmp_path, duration):
    target=tmp_path/'trial'
    with pytest.raises(ValueError):
        prepare(target,distance=5,max_duration_s=duration)
    assert not target.exists()
    save(tmp_path/'manifest.json',{'runtime_conditions':{'max_duration_s':duration}})
    with pytest.raises(ValueError):runner_timeout_s(tmp_path)


@pytest.mark.parametrize('operator,outer,selected', [(360,1860,1800),(1800,420,1800),(1800,1860,360)])
def test_stale_timeout_in_any_layer_is_rejected(operator, outer, selected):
    wrapper=f'timeout --signal=TERM --kill-after=10s {outer}s docker compose exec --max-duration-s {operator}'
    with pytest.raises(ValueError):
        validate_timing({'max_duration_s':1800},wrapper,{'max_duration_s':selected})


@pytest.mark.parametrize('stale', [None,'body','observation','camera'])
def test_recorder_cannot_silently_end_at_ten_minutes(tmp_path, stale):
    (tmp_path/'full5m').mkdir()
    for kind,name in [('body','record_body.py'),('observation','observe_camera_and_sweep.py')]:
        limit=600 if stale==kind else 2040
        (tmp_path/'full5m'/name).write_text(f'while time.monotonic()-start<{limit}: pass\n')
    camera_limit=600 if stale=='camera' else 2040
    wrapper=f'record_camera_timing.py --output /trial/full5m/camera-timing --duration-s {camera_limit}'
    conditions={'max_duration_s':1800,'recorder_max_duration_s':2040}
    if stale:
        with pytest.raises(ValueError):validate_recorder_timing(tmp_path,conditions,wrapper)
    else:
        validate_recorder_timing(tmp_path,conditions,wrapper)
