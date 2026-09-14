#!/usr/bin/env python3
"""Operator entry point for one recorded Full or Direct episode from the marked start."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time

import robot_goal_campaign as campaign
from fixed_start_goals import validate_plan, validate_start_confirmation
from robot_episode_archive import analyze
from robot_recording_integrity import check as check_recording

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CAMPAIGN = REPO / '.local-data/recording-five-goals-recaptured5-20260914'
STATE = REPO / '.local-data/saved-goal-operator'


def read(path, default=None):
    return json.loads(Path(path).read_text()) if Path(path).exists() else default


def save(path, value):
    campaign.write(path, value)


def load_campaign(root):
    c = read(root / 'campaign.json')
    if c['status'] != 'goals_sealed' or not c['same_marked_start_each_episode']:
        raise ValueError('고정 시작점의 다섯 목표가 확정된 캠페인이 아닙니다.')
    if campaign.digest(root / 'goal-plan.json') != c['goal_plan_sha256']:
        raise ValueError('저장한 목표 파일이 변경됐습니다.')
    plan = read(root / 'goal-plan.json')
    for label in ('1', '2', '3', '4', '5'):
        validate_plan(plan, label)
    if c['origin_id'] != plan['origin']['origin_id']:
        raise ValueError('목표와 시작점 식별자가 다릅니다.')
    return c, plan


def prepared_trial(root, label, navigation_mode='full'):
    if navigation_mode not in ('full', 'direct_goal'):
        raise ValueError('지원하지 않는 주행 방식입니다.')
    c, _ = load_campaign(root)
    if str(label) in c.get('pending_goal_replacements', {}):
        raise ValueError(str(label) + '번은 새 목표 좌표 등록 대기 중입니다. 새 좌표를 확정한 뒤 실행하세요.')
    if campaign.digest(c['map_database']) != c['map_sha256']:
        raise ValueError('확정한 지도 파일이 변경됐습니다.')
    for entry in reversed(c['trials']):
        if entry['goal_label'] != label or entry['method'] != navigation_mode or entry['status'] != 'prepared':
            continue
        run = Path(entry['path'])
        if any((run / marker).exists() for marker in ('USED', 'START_GOAL', 'CANCEL', 'STACK_STARTED')):
            continue
        if any(run.glob('full5m/**/STOP')):
            continue
        binding = read(run / 'campaign-binding.json', {})
        if any(binding.get(k) != c[k] for k in ('campaign_id', 'goal_plan_sha256', 'map_sha256', 'origin_id')):
            continue
        if binding.get('method') != navigation_mode:
            continue
        manifest = read(run / 'manifest.json', {})
        settings = c.get('trial_settings', {})
        conditions = manifest.get('runtime_conditions', {})
        if conditions.get('navigation', 'full') != navigation_mode:
            continue
        if (manifest.get('images', {}).get('escape') != c['navigation_image']
                or conditions.get('look_execution', 'forward_0p1') != settings.get('look', 'forward_0p1')
                or conditions.get('turn_speed_radps', .5) != settings.get('turn_speed_radps', .5)
                or conditions.get('max_duration_s', 360) != settings.get('max_duration_s', 360)):
            continue
        try:
            campaign.validate_trial(run, c['validation'])
        except (ValueError, AssertionError, subprocess.CalledProcessError):
            continue  # Preserve old prepared artifacts and create a fresh attempt.
        return entry
    return campaign.add_trial(root, label, navigation_mode)


def active_services():
    lines = subprocess.check_output([
        'docker', 'ps', '--filter', 'label=com.docker.compose.project=escape-robot-minimal',
        '--format', '{{.Label "com.docker.compose.service"}}'], text=True, timeout=10)
    return set(lines.splitlines()) & {'escape', 'unitree-command', 'pixnav', 'native-recording'}


def stop_request(run, source):
    run = Path(run)
    rt = run / 'full5m/runtime'
    rt.mkdir(parents=True, exist_ok=True)
    # Write STOP first: it is checked by the operator before enable and during driving.
    (rt / 'STOP').touch()
    (run / 'CANCEL').touch()
    p = run / 'operator-stop.json'
    if not p.exists():
        save(p, dict(source=source, requested_unix=time.time(),
                     before_final_result=not (rt / 'result.json').exists(),
                     goal_was_published=(rt / 'goal.json').exists()))


def dispatch(run, plan, confirmed_unix):
    if (run / 'CANCEL').exists() or (run / 'full5m/runtime/STOP').exists():
        return False
    confirmation = dict(fixed_start_confirmed=True, origin_id=plan['origin']['origin_id'],
                        confirmed_unix=confirmed_unix,
                        source='human_invoked_saved_goal_runner')
    validate_start_confirmation(plan, confirmation, time.time())
    save(run / 'START_GOAL', confirmation)
    return True


def recording_ready(run):
    """STACK_STARTED means processes exist, not that DDS/recorders are ready."""
    env = dict(os.environ, ESCAPE_TRIAL_DIR=str(run))
    cmd = ['docker', 'compose', '--env-file', str(REPO / 'config/robot-full.env'),
           '-f', str(REPO / 'compose.robot-minimal.yaml'),
           '-f', str(REPO / 'compose.robot-full.yaml'),
           '-f', str(run / 'compose.trial.yaml'), 'logs', '--no-color', 'native-recording']
    with (run / 'native-recording.log').open('w') as log:
        subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=5)
    report = check_recording(run, live=True)
    save(run / 'operator-recording-readiness.json', report)
    with (run / 'operator-recording-readiness.jsonl').open('a') as log:
        log.write(json.dumps(report) + '\n')
    return report['passed']


class Supervisor:
    def __init__(self, run):
        self.run = Path(run)
        self.child = None
        self.stop_at = None
        self.fallback_done = False
        self.terminated = False

    def stop(self, source='operator_signal'):
        stop_request(self.run, source)
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            print('\n정지 요청을 전달했습니다. 제어 종료와 로그 저장을 기다립니다.', flush=True)

    def signal(self, number, _frame):
        self.stop('signal_' + signal.Signals(number).name)

    def fallback(self):
        if self.stop_at is None or not self.child or self.child.poll() is not None:
            return
        age = time.monotonic() - self.stop_at
        if age > 5 and not self.fallback_done and not (self.run / 'full5m/runtime/result.json').exists():
            self.fallback_done = True
            # Independent fallback when the in-container operator no longer responds.
            env = dict(os.environ, ESCAPE_TRIAL_DIR=str(self.run))
            cmd = ['docker', 'compose', '--env-file', str(REPO / 'config/robot-full.env'),
                   '-f', str(REPO / 'compose.robot-minimal.yaml'),
                   '-f', str(REPO / 'compose.robot-full.yaml'),
                   '-f', str(self.run / 'compose.trial.yaml'),
                   'stop', '--timeout', '2', 'escape', 'unitree-command']
            with (self.run / 'operator-stop-fallback.log').open('a') as log:
                subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=20)
        if age > 20 and not self.terminated:
            self.terminated = True
            try:
                os.killpg(self.child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if age > 90:
            os.killpg(self.child.pid, signal.SIGKILL)
            raise RuntimeError('종료 시간 초과. 리모컨 정지를 유지하고 종료 로그를 확인하세요.')

    def wait(self):
        while self.child and self.child.poll() is None:
            self.fallback()
            time.sleep(.2)


def progress(line):
    try:
        d = json.loads(line)
    except ValueError:
        return
    if 'goal_published' in d and isinstance(d['goal_published'], dict):
        print('목표 발송 완료. Full 비동기 주행을 시작했습니다.', flush=True)
    elif 'elapsed_s' in d:
        remaining = d.get('goal_distance_m')
        distance = '?' if remaining is None else f'{remaining:.2f}m'
        mode = (d.get('controller') or {}).get('mode', '?')
        print(f"{d['elapsed_s']:6.0f}초 | 목표까지 {distance} | {mode}", flush=True)


def runner_timeout_s(run):
    # Older prepared profiles had a 360s episode / 540s supervisor limit.
    duration = read(Path(run) / 'manifest.json', {}).get('runtime_conditions', {}).get('max_duration_s', 360)
    if isinstance(duration, bool) or not math.isfinite(duration) or not 0 < duration <= 1800:
        raise ValueError('Invalid recorded episode time limit')
    return duration + 180  # Preserve bounded startup and recorder shutdown time.


def required_recording_space_bytes(duration):
    # Recent Full episodes recorded 22-28 MB/s, mainly raw cloud/camera MCAP.
    # Keep the existing reserve and allow 30 MiB/s for newly extended episodes.
    reserve = 10 * 1024**3
    return reserve if duration <= 360 else reserve + math.ceil(duration * 30 * 1024**2)


def execute(run, plan, confirmed_unix, *, start_goal=True):
    overall_timeout = runner_timeout_s(run)
    supervisor = Supervisor(run)
    old = {s: signal.signal(s, supervisor.signal) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    save(STATE / 'active.json', dict(run=str(run), pid=os.getpid(), started_unix=time.time()))
    output = run / 'operator-console.log'
    started = time.monotonic()
    dispatched = False
    cursor = 0
    next_probe = 0.
    ready_count = 0
    try:
        with output.open('x') as log:
            supervisor.child = subprocess.Popen(['bash', str(run / 'run_trial.sh')], cwd=REPO,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            print('카메라·정책·센서 기록과 제어기를 준비합니다. Ctrl+C: 정지', flush=True)
            while supervisor.child.poll() is None:
                if supervisor.stop_at is None and (run / 'CANCEL').exists():
                    supervisor.stop('external_stop_command')
                if (not dispatched and supervisor.stop_at is None and (run / 'STACK_STARTED').exists()
                        and time.monotonic() >= next_probe):
                    ready_count = ready_count + 1 if recording_ready(run) else 0
                    next_probe = time.monotonic() + 1.
                    if ready_count >= 2:
                        if start_goal:
                            dispatched = dispatch(run, plan, confirmed_unix)
                            if dispatched:
                                print('영상·센서 기록 수신 확인. 제어 상태 확인 후 자동 출발합니다.', flush=True)
                        else:
                            save(run / 'no-motion-startup-validation.json',
                                 dict(recording_ready=True, consecutive_passes=ready_count,
                                      goal_dispatch_allowed=False, checked_unix=time.time()))
                            supervisor.stop('no_motion_startup_validation_complete')
                    elif int(time.monotonic() - started) % 5 == 0:
                        print('영상·센서 기록 연결을 기다리는 중입니다. 아직 출발하지 않았습니다.', flush=True)
                p = run / 'full5m/drive-console.log'
                if p.exists():
                    with p.open() as stream:
                        stream.seek(cursor)
                        while True:
                            position = stream.tell()
                            line = stream.readline()
                            if not line or not line.endswith('\n'):
                                cursor = position
                                break
                            progress(line)
                elapsed = time.monotonic() - started
                if supervisor.stop_at is None and ((not dispatched and elapsed > 100) or elapsed > overall_timeout):
                    supervisor.stop('runner_timeout')
                supervisor.fallback()
                time.sleep(.2)
            return supervisor.child.returncode
    finally:
        if supervisor.child and supervisor.child.poll() is None:
            supervisor.stop('runner_exit')
            supervisor.wait()
        for s, handler in old.items():
            signal.signal(s, handler)
        # Keep this marker if motion service shutdown cannot be confirmed.
        if not active_services():
            (STATE / 'active.json').unlink(missing_ok=True)


def field_report(run):
    field = read(run / 'field-report.json')
    request = read(run / 'operator-stop.json', {})
    if (request.get('before_final_result') and request.get('goal_was_published')
            and request.get('source') in ('signal_SIGINT', 'signal_SIGTERM', 'signal_SIGHUP', 'external_stop_command')):
        field.update(status='received_partial', direct_intervention=True,
                     operator_comment='Operator runner stop requested: ' + request['source'])
    if sys.stdin.isatty() and not request:
        print('현장 기록: 개입/접촉/진로방해 순서로 0=없음, 1=있음, ?=모름. 예: 0 0 0 | 메모')
        try:
            print('Enter 또는 20초 무응답이면 미확인으로 보존: ', end='', flush=True)
            answer = sys.stdin.readline().strip() if select.select([sys.stdin], [], [], 20)[0] else ''
        except (EOFError, KeyboardInterrupt):
            answer = ''
        flags, _, comment = answer.partition('|')
        values = flags.split()
        if len(values) == 3 and all(v in ('0', '1', '?') for v in values):
            field.update(zip(('direct_intervention', 'contact', 'obstruction'),
                             [None if v == '?' else v == '1' for v in values]))
            field.update(status='received', operator_comment=comment.strip(), measurement_method='operator_report')
        elif answer:
            field.update(status='received_partial', operator_comment=answer)
    save(run / 'field-report.json', field)


def finish(root, entry, returncode):
    run = Path(entry['path'])
    result = read(run / 'full5m/runtime/result.json', {})
    if active_services():
        raise RuntimeError('제어/기록 서비스가 아직 실행 중입니다. 리모컨 정지를 유지하세요. ' + str(run))
    if result.get('goal_published') and result.get('shutdown_service_confirmed') is not True:
        print('주의: 제어 서비스는 종료됐지만 StopMove 서비스 응답은 미확인입니다.', flush=True)
    print('제어 종료. 결과: ' + result.get('reason', '목표 발송 전 종료'), flush=True)
    if result.get('goal_distance_m') is not None:
        print(f"남은 거리(오도메트리): {result['goal_distance_m']:.2f}m", flush=True)
    print('원본 로그: ' + str(run), flush=True)
    if result.get('goal_published'):
        field_report(run)
    else:
        report = read(run / 'recording-preflight.json') or read(run / 'operator-recording-readiness.json', {})
        for issue in report.get('issues', []):
            print('시작 전 종료 원인: ' + issue, flush=True)
        save(run / 'prestart-outcome.json', dict(physical_episode_started=False,
             result_reason=result.get('reason', 'PRESTART_EXIT'),
             readiness_issues=report.get('issues', []), runner_returncode=returncode))
    if not (run / 'RECORDERS_CLOSED').exists():
        raise RuntimeError('기록 종료를 확인하지 못했습니다. 원본은 보존했으며 기록 완료로 처리하지 않습니다.')
    archived = campaign.archive_trial(root, entry['trial_id'])
    summary, _ = analyze(run)
    summary['runner_returncode'] = returncode
    summary['final_goal_distance_m'] = result.get('goal_distance_m')
    summary['recording_complete'] = archived['recording_complete']
    summary['archive'] = archived['archive']
    summary['trial_id'] = entry['trial_id']
    summary['goal_label'] = entry['goal_label']
    summary['method'] = entry['method']
    summary['arrival_mode'] = ('operator_intervention' if summary['field_report'].get('direct_intervention')
                               else 'distance_threshold' if result.get('reason') == 'GOAL_DISTANCE_REACHED'
                               else 'not_reached')
    with (root / 'operator-results.jsonl').open('a') as out:
        out.write(json.dumps(summary, ensure_ascii=False) + '\n')
    print('궤적 CSV·결과·원본 해시: ' + archived['archive'], flush=True)
    if not archived['recording_complete']:
        print('일부 기록 검사 실패. recording-integrity.json에 누락 내역을 남겼습니다.', flush=True)
        return 3
    print('로그 저장 완료. 다음 번호도 표시한 시작 위치·방향으로 옮긴 뒤 별도로 실행하세요.', flush=True)
    return returncode


def main():
    parser = argparse.ArgumentParser(description='표시한 시작점 (0,0), 시작 방향 +X에서 기록용 목표 한 개 실행')
    parser.add_argument('goal', nargs='?', choices=('1', '2', '3', '4', '5'))
    parser.add_argument('--root', type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument('--navigation-mode', choices=('full', 'direct_goal'), default='full')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--list', action='store_true', help='목표 좌표만 표시; 주행 없음')
    mode.add_argument('--check', action='store_true', help='다섯 목표 실행 파일 검증·준비; 주행 없음')
    mode.add_argument('--stop', action='store_true', help='다른 터미널에서 현재 회차 정지 요청')
    args = parser.parse_args()
    STATE.mkdir(parents=True, exist_ok=True)
    if args.stop:
        active = read(STATE / 'active.json')
        if not active:
            print('이 실행기로 진행 중인 회차가 없습니다.')
            return 0
        stop_request(Path(active['run']), 'external_stop_command')
        print('정지 요청 전달 완료: ' + active['run'])
        return 0
    root = args.root.resolve()
    c, plan = load_campaign(root)
    if args.list:
        for g in plan['goals']:
            if g['label'] in c.get('pending_goal_replacements', {}):
                print(g['label'] + '번: 새 목표 좌표 등록 대기 (4번 아래 새 지점)')
                continue
            xy = g['point_goal_xy_m']
            print(f"{g['label']}번: x={xy['x']:.3f}m, y={xy['y']:.3f}m (앞 +X, 왼쪽 +Y)")
        return 0
    with (STATE / 'runner.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('이미 실행 중인 회차가 있습니다. 중복 출발하지 않습니다.')
        if args.check:
            waiting = []
            for label in ('1', '2', '3', '4', '5'):
                if label in c.get('pending_goal_replacements', {}):
                    waiting.append(label)
                    print(label + '번: 새 목표 좌표 등록 대기; 실행 파일을 준비하지 않았습니다.', flush=True)
                    continue
                entry = prepared_trial(root, label, args.navigation_mode)
                print(label + '번 실행 파일 검증 완료: ' + entry['trial_id'], flush=True)
            if waiting:
                print('등록된 목표 검사 완료. ' + ', '.join(waiting) + '번은 등록 대기입니다. 로봇/ROS를 시작하거나 목표를 발송하지 않았습니다.')
                return 2
            print('파일·설정 검증 완료. 로봇/ROS를 시작하거나 목표를 발송하지 않았습니다.')
            return 0
        if not sys.stdin.isatty():
            raise ValueError('주행은 운영자가 연결한 터미널에서 실행하세요. 파일 검증은 --check를 사용하세요.')
        label = args.goal or input('실행할 목표 번호 (1~5, 종료 Enter): ').strip()
        if not label:
            return 0
        if label not in ('1', '2', '3', '4', '5'):
            raise ValueError('목표 번호는 1~5입니다.')
        confirmed = time.time()  # Actual operator invocation/selection; never refresh while waiting.
        if active_services():
            raise RuntimeError('기존 주행/기록 서비스가 실행 중입니다. 먼저 해당 회차를 종료하세요.')
        duration = c.get('trial_settings', {}).get('max_duration_s', 360)
        required_space = required_recording_space_bytes(duration)
        if shutil.disk_usage(root).free < required_space:
            raise RuntimeError(f'{duration:g}초 회차의 로그 저장 예상 여유 공간 {required_space/1024**3:.1f}GiB가 필요합니다. 이전 로그를 보존한 채 공간을 확보하세요.')
        print(f'{label}번 준비: 표시한 (0,0)과 원래 방향에 배치하고, 공간·비상정지를 확보한 상태에서 실행하는 명령입니다.', flush=True)
        entry = prepared_trial(root, label, args.navigation_mode)
        c, plan = load_campaign(root)
        for name, sha in c['camera']['sha256'].items():
            if campaign.digest(root / name) != sha:
                raise ValueError('카메라 설정이 변경됐습니다: ' + name)
        run = Path(entry['path'])
        conditions = read(run / 'manifest.json')['runtime_conditions']
        look_cm = {'forward_0p05':5, 'forward_0p1':10, 'forward_0p2':20}[conditions['look_execution']]
        method_name = 'Full 비동기' if args.navigation_mode == 'full' else 'Direct PixelNav (VLM 호출 없음)'
        print(f"{method_name} / 룩 동작 {look_cm}cm 전진 대체 / 전진 0.5m/s / 회전 {conditions['turn_speed_radps']}rad/s / 도착 반경 1m / 최대 {conditions['max_duration_s']:g}초", flush=True)
        save(run / 'operator-invocation.json', dict(goal_label=label, navigation_mode=args.navigation_mode, confirmed_unix=confirmed,
             runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             coordinate_contract='Same marked physical start and heading; capture current odom and reanchor once'))
        shutil.copy2(__file__, run / 'run_saved_robot_goal.snapshot.py')
        with (run / 'camera-startup.log').open('w') as log:
            subprocess.run(['docker', 'compose', '-f', c['camera']['compose'], 'up', '-d', '--no-deps', 'camera'],
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=30)
        print('회차: ' + entry['trial_id'], flush=True)
        try:
            code = execute(run, plan, confirmed)
        except Exception as exc:
            save(run / 'operator-error.json', dict(error=str(exc), unix=time.time()))
            print('실행 오류: ' + str(exc), file=sys.stderr)
            code = 1
        return finish(root, entry, code)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print('\n종료: ' + str(exc), file=sys.stderr)
        sys.exit(1)
