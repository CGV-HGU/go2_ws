#!/usr/bin/env python3
"""Inventory recording sizes only. Never deletes or modifies source recordings."""
import csv
import datetime
import hashlib
import json
import os
from pathlib import Path

ROOT = Path('/home/unitree/s2e-vlm-async-framework-minimal/.local-data')
OUT = Path(__file__).resolve().parent
CAMPAIGNS = [
    'recording-five-goals-20260913',
    'recording-five-goals-long5-20260913',
    'recording-five-goals-recaptured5-20260914',
]
CANDIDATES = [
    'full-stack-soak/qualification01/native-bag',
    'full-stack-soak/setup01/native-bag',
    'concurrent-load-cyclone/bag',
    'concurrent-load/bag',
]


def files(path):
    for base, dirs, names in os.walk(str(path), followlinks=False):
        dirs[:] = [d for d in dirs if not Path(base, d).is_symlink()]
        for name in names:
            p = Path(base, name)
            if p.is_file() and not p.is_symlink():
                yield p


def size(path):
    logical = allocated = count = 0
    for p in files(path):
        s = p.stat()
        logical += s.st_size
        allocated += s.st_blocks * 512
        count += 1
    return dict(files=count, bytes=logical, allocated_file_bytes=allocated)


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(part)
    return h.hexdigest()


def dump(name, data):
    (OUT / name).write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')


def main():
    rows = []
    for campaign in CAMPAIGNS:
        for episode in sorted((ROOT / campaign / 'episodes').iterdir()):
            if not episode.is_dir():
                continue
            groups = dict(native_bag=0, observation_images=0, observation_events=0,
                          policy_inputs=0, policy_trace=0, body_events=0,
                          camera_timing=0, runtime_events=0, other=0)
            for p in files(episode):
                rel = p.relative_to(episode).as_posix()
                if rel.startswith('native-inputs/'):
                    key = 'native_bag'
                elif rel.startswith('full5m/observation-recording/'):
                    key = 'observation_images' if p.suffix.lower() in ('.ppm', '.png', '.jpg', '.jpeg') else 'observation_events'
                else:
                    key = next((label for prefix, label in [
                        ('full5m/policy-inputs/', 'policy_inputs'),
                        ('full5m/policy-trace/', 'policy_trace'),
                        ('full5m/body-recording/', 'body_events'),
                        ('full5m/camera-timing/', 'camera_timing'),
                        ('full5m/runtime/', 'runtime_events'),
                    ] if rel.startswith(prefix)), 'other')
                groups[key] += p.stat().st_size
            result_file = episode / 'full5m/runtime/result.json'
            result = json.loads(result_file.read_text()) if result_file.exists() else {}
            manifest_file = episode / 'manifest.json'
            manifest = json.loads(manifest_file.read_text()) if manifest_file.exists() else {}
            conditions = manifest.get('runtime_conditions', {})
            rows.append(dict(campaign=campaign, episode=episode.name,
                             result=result.get('reason', ''),
                             navigation=conditions.get('navigation', ''),
                             timeout_s=manifest.get('max_duration_s', ''),
                             total_bytes=sum(groups.values()), **groups))
    with (OUT / 'episode_storage.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    base = ROOT / 'jetson-comprehensive-audit-20260913'
    candidates = []
    for name in CANDIDATES:
        path = base / name
        listing = []
        for p in sorted(files(path)):
            st = p.stat()
            entry = dict(path=str(p), bytes=st.st_size, mtime_ns=st.st_mtime_ns)
            # This audit does not claim a full read/hash of large bag payloads.
            if p.suffix in ('.yaml', '.json'):
                entry['sha256'] = sha(p)
            listing.append(entry)
        candidates.append(dict(path=str(path), classification='generated_offline_load_test_bag',
                               action='candidate_only_not_deleted', **size(path), inventory=listing))
    preserved_evidence = []
    for parent in ('full-stack-soak/qualification01', 'full-stack-soak/setup01',
                   'concurrent-load-cyclone', 'concurrent-load'):
        for p in sorted((base / parent).iterdir()):
            if p.is_file() and p.suffix in ('.json', '.py', '.sh') and p.stat().st_size < 2 * 1024**2:
                preserved_evidence.append(dict(path=str(p), bytes=p.stat().st_size, sha256=sha(p)))
    dump('cleanup_candidates.json', dict(source_bags_deleted=False,
         candidates=candidates, total_candidate_bytes=sum(x['bytes'] for x in candidates),
         supporting_diagnostics_to_preserve=preserved_evidence,
         limitation='Generated payload can be regenerated, but an exact old transport trace cannot; keep historical reports including failed tests. No large payload hash pass performed.'))

    top = [dict(path=str(p), **size(p)) for p in ROOT.iterdir() if p.is_dir() and not p.is_symlink()]
    top.sort(key=lambda x: x['bytes'], reverse=True)
    v = os.statvfs(str(ROOT))
    dump('storage_inventory.json', dict(checked_at=datetime.datetime.now().astimezone().isoformat(),
         source_root=str(ROOT), source_modified=False, usage_method='os.walk/stat, symlinks excluded; logical sizes and allocated file blocks; directory blocks not counted',
         filesystem=dict(total_bytes=v.f_blocks*v.f_frsize, available_bytes=v.f_bavail*v.f_frsize),
         local_data=top, campaign_episode_totals={c: sum(r['total_bytes'] for r in rows if r['campaign']==c) for c in CAMPAIGNS},
         result_files=sum(bool(r['result']) for r in rows), episode_directories=len(rows)))
    print(json.dumps(dict(episode_directories=len(rows), result_files=sum(bool(r['result']) for r in rows),
                     available_bytes=v.f_bavail*v.f_frsize,
                     candidate_bytes=sum(x['bytes'] for x in candidates)), indent=2))


if __name__ == '__main__':
    main()
