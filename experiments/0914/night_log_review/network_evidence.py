#!/usr/bin/env python3
"""Extract relevant existing host logs without altering network state."""
import json
import re
from pathlib import Path
from datetime import datetime, timezone, timedelta
from collections import Counter

OUT = Path(__file__).resolve().parent/'network'
KST = timezone(timedelta(hours=9))
OUT.mkdir(parents=True,exist_ok=True)


def clean(line):
    # Hardware/network identity is unnecessary for sharing these timing events.
    line = re.sub(r'\b(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}\b','[MAC]',line)
    return re.sub(r'\b[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}\b','[UUID]',line)


events=[]
for name in ['syslog','kern.log']:
    source=Path('/var/log')/name
    extracted=[]
    with source.open(errors='replace') as f:
        for no,line in enumerate(f,1):
            if not line.startswith(('Sep 13 ','Sep 14 ')):
                continue
            kind=None
            if name=='syslog' and ('NetworkManager' in line or 'wpa_supplicant' in line):
                if ('wlan0' in line or 'Ultra' in line or 'HGU_WLAN' in line) and re.search(
                    r'CTRL-EVENT-DISCONNECTED|Activation: (starting|failed|successful)|Connected to wireless|link timed out|state change:.*(failed|removed)|default for IPv4|Trying to associate|CTRL-EVENT-REGDOM-CHANGE',line):
                    kind='wifi'
            if name=='kern.log' and re.search(r'usb 1-2:|rtl8851|8851bu|wlan0',line) and re.search(
                r'USB disconnect|reset .*USB|new high-speed|idVendor=|Product:|Manufacturer:|timed out|error -|failed',line):
                kind='usb'
            if kind:
                # Old syslog lacks year; the selected experimental date is 2026.
                when=datetime.strptime('2026 '+line[:15],'%Y %b %d %H:%M:%S').replace(tzinfo=KST)
                row=dict(source=str(source),line=no,kst=when.isoformat(),unix_s=when.timestamp(),kind=kind,text=clean(line.rstrip()))
                events.append(row)
                extracted.append(f'{no}: '+clean(line.rstrip()))
    (OUT/(name+'.excerpt.txt')).write_text('\n'.join(extracted)+'\n')
(OUT/'events.json').write_text(json.dumps(events,ensure_ascii=False,indent=2)+'\n')
late=[e for e in events if '2026-09-14T00:' <= e['kst'] < '2026-09-14T01:04:00']
summary=dict(source_note='syslog year inferred from experimental date; boot-time wall-clock resets exist; retain raw lines and kernel monotonic times',
             event_count=len(events),late_usb_disconnect_count=sum('USB disconnect' in e['text'] for e in late),
             late_wifi_disconnects=[e for e in late if 'CTRL-EVENT-DISCONNECTED' in e['text']],
             late_connection_failures=[e for e in late if "Activation: failed for connection" in e['text']],
             association_during_latest_full_trials='Ultra, active from 00:51:04; disconnected at 00:57:50; failed ssid-not-found at 00:58:06',
             causal_limit='Timeout request timing does not separate Wi-Fi, VPN, server queue or model execution. Operator confirmed unplugging/replugging the USB adapter; exact times unknown. USB removal count is not spontaneous failure count.',
             operator_replugged_usb_wifi=True,
             current_readonly_observation={'date':'2026-09-14','wifi_power_save':'on','usb_1_2_power_control':'on','usb_1_2_autosuspend_delay_ms':-1000},
             interpretation='USB runtime autosuspend is disabled for this device; Wi-Fi power save is enabled. Neither identifies the historical root cause alone.')
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'events':len(events),'late_usb_disconnects':summary['late_usb_disconnect_count']}))
