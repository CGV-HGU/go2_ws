"""Campaign identity, retry preservation and recording-loss regressions; no ROS."""
import json
from pathlib import Path
import sqlite3
import time
import pytest

import robot_goal_campaign as campaign
from fixed_start_goals import make_origin, capture_goal
from robot_recording_integrity import check, NATIVE_TOPICS
import robot_recording_integrity as recording


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n')


def test_live_recording_clock_is_sampled_after_concurrent_file_reads(tmp_path,monkeypatch):
    image=tmp_path/'full5m/observation-recording/rgb.ppm'
    image.parent.mkdir(parents=True);image.write_bytes(b'P6\n1 1\n255\n\0\0\0')
    (tmp_path/'native-recording.log').write_text('\n'.join(
        "Subscribed to topic '"+topic+"'" for topic in NATIVE_TOPICS))
    clock=[100.]
    def advancing_rows(path,issues,live=False):
        clock[0]+=.2
        kinds=(['low','sport'] if 'body-recording' in str(path) else
               ['raw_image_header','time_sync_status'] if 'camera-timing' in str(path)
               else ['image'])
        return [dict(event=k,wall_ns=int(clock[0]*1e9),data={'path':'rgb.ppm'}) for k in kinds]
    monkeypatch.setattr(recording,'rows',advancing_rows)
    monkeypatch.setattr(recording.time,'time',lambda:clock[0])
    assert check(tmp_path,live=True)['passed']
    stale=check(tmp_path,live=True,now=110.)
    assert not stale['passed']
    assert 'stale recorded camera:raw_image_header' in stale['issues']


@pytest.fixture
def collected(tmp_path,monkeypatch):
    monkeypatch.setattr(campaign,'prepare_map',lambda p,image:p.mkdir())
    monkeypatch.setattr(campaign,'prepare_camera',lambda p:{'test_only':True})
    validation=tmp_path/'validation.json';save(validation,dict(offline_checks_passed=True,navigation_image='unused',navigation_image_id='image'))
    root=tmp_path/'campaign';campaign.prepare_campaign(root,validation)
    db=root/'mapping/map.db';db.write_bytes(b'frozen-map-for-file-contract-test')
    now=time.time()+1
    save(root/'mapping/manifest.json',dict(status='collecting_fixed_start_goals',host_boot_id='boot',
         source_database=str(db),source_database_sha256=campaign.digest(db),localization_mode='fixed_start_odometry'))
    def snapshot(x):
        return dict(host_boot_id='boot',checked_unix=now,odom_publisher_gids=['owner'],
            topics=dict(odom=dict(frame='odom',stamp_unix=now,received_unix=now,
            pose=dict(pose=dict(position=dict(x=x,y=0.,z=.3),orientation=dict(x=0.,y=0.,z=0.,w=1.))))))
    def with_evidence(record,name,raw):
        save(root/'goals'/(name+'.snapshot.json'),raw)
        record.update(source_snapshot_file=name+'.snapshot.json',source_snapshot_sha256=campaign.digest(root/'goals'/(name+'.snapshot.json')))
        save(root/'goals'/(name+'.json'),record);return record
    origin=with_evidence(make_origin(snapshot(0),now,'boot'),'origin',snapshot(0))
    for label in range(1,6):with_evidence(capture_goal(origin,str(label),snapshot(label+1.),now,'boot'),str(label),snapshot(label+1.))
    return root


def fake_prepare(out,**kw):
    out.mkdir();campaign.write(out/'manifest.json',dict(sha256={},images={'escape':kw['navigation_image']},runtime_conditions={
        'navigation':kw['navigation_mode'],'look_execution':kw.get('look','forward_0p1'),
        'turn_speed_radps':kw.get('turn_speed_radps',.5)}))
    (out/'goal-plan.json').write_bytes(Path(kw['fixed_start_plan']).read_bytes())


def test_five_goals_share_frozen_map_and_full_direct_targets_and_retries(collected,monkeypatch):
    root=collected;campaign.bind_goals(root)
    monkeypatch.setattr(campaign,'prepare_trial',fake_prepare)
    monkeypatch.setattr(campaign,'validate_trial',lambda *a:None)
    trials=[campaign.add_trial(root,str(i)) for i in range(1,6)]
    assert len({t['trial_id'] for t in trials})==5
    assert {t['goal_label'] for t in trials}=={'1','2','3','4','5'}
    direct=campaign.add_trial(root,'3','direct_goal')
    assert (Path(direct['path'])/'goal-plan.json').read_bytes()==(Path(trials[2]['path'])/'goal-plan.json').read_bytes()
    retry=campaign.add_trial(root,'3');assert retry['trial_id']=='full-goal-3-002'
    assert Path(trials[2]['path']).exists()


@pytest.mark.parametrize('fault',['fifth_missing','map_changed','old_origin','snapshot_changed','duplicate_goal'])
def test_partial_or_mixed_goal_set_is_not_sealed(collected,fault):
    root=collected
    if fault=='fifth_missing':(root/'goals/5.json').unlink()
    if fault=='map_changed':(root/'mapping/map.db').write_bytes(b'other-map')
    if fault=='old_origin':
        p=root/'goals/origin.json';v=campaign.read(p);v['captured_unix']=0;save(p,v)
    if fault=='snapshot_changed':(root/'goals/3.snapshot.json').write_text('{}')
    if fault=='duplicate_goal':
        p=root/'goals/5.json';v=campaign.read(p);v['label']='4';save(p,v)
    with pytest.raises(ValueError):campaign.bind_goals(root)
    assert campaign.read(root/'campaign.json')['status']=='awaiting_mapping'


def test_failed_episode_and_incomplete_recording_are_retained(collected,monkeypatch):
    root=collected;campaign.bind_goals(root)
    monkeypatch.setattr(campaign,'prepare_trial',fake_prepare);monkeypatch.setattr(campaign,'validate_trial',lambda *a:None)
    trial=campaign.add_trial(root,'1');run=Path(trial['path']);rt=run/'full5m/runtime';rt.mkdir(parents=True)
    events=[dict(event='goal_published',elapsed_s=1,data={}),dict(event='task',elapsed_s=1.1,data={'episode_id':'failed-episode'})]
    (rt/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    save(rt/'result.json',dict(goal_published=True,reason='TRIAL_TIME_LIMIT',shutdown_service_confirmed=True))
    with pytest.raises(ValueError,match='recorders'):campaign.archive_trial(root,trial['trial_id'])
    (run/'RECORDERS_CLOSED').touch()
    result=campaign.archive_trial(root,trial['trial_id'])
    assert result['started'] is True and result['recording_complete'] is False
    assert result['episode_id']=='failed-episode' and result['result_reason']=='TRIAL_TIME_LIMIT'
    assert campaign.read(Path(result['archive'])/'episode.json')['success']==0
    assert (Path(result['archive'])/'raw-manifest.json').exists()
    with pytest.raises(ValueError,match='already archived'):campaign.archive_trial(root,trial['trial_id'])


@pytest.mark.parametrize('fault',[None,'body_stale','empty_cloud_subscription','missing_rgb_bytes'])
def test_preflight_requires_actual_fresh_recordings(tmp_path,fault):
    now=100.
    def stream(path,values):
        p=tmp_path/path;p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(''.join(json.dumps(v)+'\n' for v in values))
    stream('full5m/body-recording/events.jsonl',[dict(kind=k,wall_ns=int((95. if fault=='body_stale' else now)*1e9)) for k in ('low','sport')])
    stream('full5m/camera-timing/events.jsonl',[dict(event=k,wall_ns=int(now*1e9)) for k in ('raw_image_header','time_sync_status')])
    stream('full5m/observation-recording/events.jsonl',[dict(event='image',wall_ns=int(now*1e9),data={'path':'rgb.ppm'})])
    if fault!='missing_rgb_bytes':(tmp_path/'full5m/observation-recording/rgb.ppm').write_bytes(b'P6\n1 1\n255\n\0\0\0')
    topics=[t for t in NATIVE_TOPICS if not (fault=='empty_cloud_subscription' and t.endswith('/cloud'))]
    (tmp_path/'native-recording.log').write_text('\n'.join("Subscribed to topic '"+t+"'" for t in topics))
    r=check(tmp_path,live=True,now=now)
    assert r['passed'] is (fault is None)


def test_fixed_start_freeze_preserves_database_and_leaves_sensor_recorders_running(tmp_path,monkeypatch):
    import robot_map_session as mapping
    db=sqlite3.connect(str(tmp_path/'mapping.db'));db.execute('create table Node(id integer)');db.execute('insert into Node values(1)');db.commit();db.close()
    calls=[];monkeypatch.setattr(mapping,'command',lambda *args:calls.append(args))
    monkeypatch.setattr(mapping.subprocess,'check_output',lambda args,**kw:json.dumps([{'State':{'Running':False,'ExitCode':0}}]).encode() if args[:2]==['docker','inspect'] else 'container-id')
    monkeypatch.setattr(mapping,'set_state',lambda run,m,status:dict(m,status=status))
    r=mapping.freeze_fixed_start(tmp_path,dict(status='mapping',host_boot_id='boot'),'boot')
    assert r['status']=='collecting_fixed_start_goals' and r['physical_localization_verified'] is False
    assert calls==[(tmp_path,'compose.json','stop','slam')]
    with sqlite3.connect(str(tmp_path/'map.db')) as saved:assert saved.execute('select id from Node').fetchone()==(1,)
    with pytest.raises(ValueError):mapping.freeze_fixed_start(tmp_path,dict(status='mapping',host_boot_id='boot'),'boot')


def test_postflight_detects_recorder_that_died_during_episode(tmp_path):
    body=tmp_path/'full5m/body-recording/events.jsonl';body.parent.mkdir(parents=True)
    body.write_text(json.dumps(dict(kind='low',wall_ns=100_000_000_000))+'\n')
    rt=tmp_path/'full5m/runtime';rt.mkdir(parents=True)
    events=[dict(event='goal_published',utc='1970-01-01T00:01:40+00:00',data={}),
            dict(event='direct_stop',utc='1970-01-01T00:01:50+00:00',data={})]
    (rt/'events.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in events))
    r=check(tmp_path)
    assert not r['passed']
    assert any('span episode: body:low' in issue for issue in r['issues'])
