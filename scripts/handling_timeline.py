#!/usr/bin/env python3
"""Compare laptop/Pi handling logs without hiding dropouts or clock uncertainty.

All times are shown in UTC where parsable. Events collected while Pi clock was
wrong can be out of order; use boot ID, uptime, and local monotonic for causality.
"""

import argparse
from collections import Counter, defaultdict
import datetime as dt
import json
from pathlib import Path
import re

TOPICS = {
    '/scan': 'scan',
    '/oakd/rgb/image_raw/compressed': 'rgb',
    '/oakd/stereo/image_raw/compressedDepth': 'depth',
}
THRESHOLDS = {'scan': 5.0, 'rgb': 3.0, 'depth': 3.0}
JOURNAL_RE = re.compile(
    r'usb|xhci|networkmanager|hostapd|wpa_supplicant|turtlebot|rplidar|'
    r'x_link|depthai|under.?voltage|throttl|brownout|boot|power', re.I)
PING_RE = re.compile(r'(\d+) packets transmitted,\s*(\d+) received,.*?([\d.]+)% packet loss')
RTT_RE = re.compile(r'(?:rtt|round-trip) min/avg/max/(?:mdev|stddev)\s*=\s*'
                    r'([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+)\s*ms')
RSSI_RE = re.compile(r'signal:\s*(-?\d+)\s*dBm')


def timestamp(text):
    """UTC lexical sort key if ISO timestamp is parseable, else raw string."""
    value = str(text or '').strip()
    try:
        moment = dt.datetime.fromisoformat(value.replace('Z', '+00:00'))
        if moment.tzinfo is None:
            return value
        return moment.astimezone(dt.timezone.utc).isoformat(timespec='milliseconds')
    except ValueError:
        return value


def ping_info(output):
    loss_match = PING_RE.search(output)
    rtt_match = RTT_RE.search(output)
    info = {'loss': None, 'received': None, 'sent': None,
            'avg_ms': None, 'max_ms': None}
    if loss_match:
        info.update(sent=int(loss_match.group(1)), received=int(loss_match.group(2)),
                    loss=float(loss_match.group(3)))
    if rtt_match:
        info.update(avg_ms=float(rtt_match.group(2)), max_ms=float(rtt_match.group(3)))
    return info


def category(hz, threshold):
    if hz is None:
        return 'UNKNOWN'
    if hz <= 0.001:
        return 'ZERO'
    if hz < threshold:
        return 'LOW'
    return 'OK'


def parse_logs(paths):
    events = []
    totals = Counter()
    summary = defaultdict(Counter)
    boots = defaultdict(set)
    network_before = {}
    last_rates = defaultdict(dict)
    for path in paths:
        p = Path(path)
        if not p.exists():
            events.append(('', p.name, 0, None, 'WARNING', 'missing file; evidence unavailable'))
            continue
        for number, line in enumerate(p.read_text(errors='replace').splitlines(), 1):
            if p.suffix != '.jsonl':
                if JOURNAL_RE.search(line):
                    first = line.split(maxsplit=1)[0] if line.strip() else ''
                    events.append((timestamp(first), p.name, number, None,
                                   'PREVIOUS_BOOT_JOURNAL', line[:600]))
                continue
            try:
                r = json.loads(line)
            except (ValueError, TypeError):
                totals['bad_json_lines'] += 1
                continue
            if not isinstance(r, dict):
                continue
            kind = r.get('kind')
            host = str(r.get('role') or ('pi' if p.name.startswith('pi-') else
                                       'laptop' if 'laptop' in p.name else 'marker'))
            time_key = timestamp(r.get('time'))
            mono = r.get('monotonic')
            detail = None
            label = str(kind).upper()
            if kind in ('start', 'heartbeat'):
                bid = r.get('boot_id')
                if bid and host != 'marker':
                    boots[host].add(str(bid))
                if kind == 'start':
                    detail = f'role={r.get("role")} boot={bid} pid={r.get("pid")} '
                    detail += f'ROS={r.get("ros_enabled")} camera_counting={r.get("camera_counting")}'
                elif host == 'pi' and (not events or number == 2):
                    detail = f'boot={bid} uptime={r.get("uptime_seconds")}'
            elif kind == 'marker':
                detail = f'trial={r.get("test", "?")} {r.get("note", "")}'
            elif kind == 'network':
                ping = ping_info(r.get('ping', {}).get('output', ''))
                link = r.get('link', {}).get('output', '')
                associated = 'Connected to' in link and 'Not connected' not in link
                rssi_match = RSSI_RE.search(link)
                rssi = int(rssi_match.group(1)) if rssi_match else None
                if ping['loss'] is None:
                    status = ('ERROR' if r.get('ping', {}).get('rc') != 0 else 'UNKNOWN')
                elif ping['loss'] >= 100.0:
                    status = 'DOWN'
                elif ping['loss'] > 0.0:
                    status = 'PARTIAL'
                else:
                    status = 'OK'
                high_latency = (ping['max_ms'] is not None and ping['max_ms'] >= 150.0)
                state = (status, associated, high_latency)
                summary[host]['network_samples'] += 1
                summary[host]['network_' + status] += 1
                if high_latency:
                    summary[host]['network_high_latency'] += 1
                previous = network_before.get(str(p))
                if state != previous or status in ('DOWN', 'PARTIAL', 'ERROR') or high_latency:
                    loss = f'{ping["loss"]:.1f}%' if ping['loss'] is not None else 'unknown'
                    avg = f'{ping["avg_ms"]:.1f}' if ping['avg_ms'] is not None else '?'
                    max_ms = f'{ping["max_ms"]:.1f}' if ping['max_ms'] is not None else '?'
                    detail = (f'ping={status} loss={loss} rtt_avg/max={avg}/{max_ms}ms '
                              f'associated={associated} signal={rssi if rssi is not None else "?"}dBm '
                              f'latency_spike={high_latency}')
                    if status == 'ERROR':
                        detail += ' error=' + r.get('ping', {}).get('output', '')[:150].replace('\n', ' ')
                network_before[str(p)] = state
            elif kind == 'rates':
                hz = r.get('hz', {})
                if not isinstance(hz, dict):
                    continue
                fields = []
                state_changes = []
                for full, short in TOPICS.items():
                    if full not in hz:
                        continue
                    try:
                        val = float(hz[full])
                    except (ValueError, TypeError):
                        continue
                    flag = category(val, THRESHOLDS[short])
                    fields.append(f'{short}={val:.2f}Hz({flag})')
                    summary[host][f'{short}_{flag}'] += 1
                    old = last_rates[str(p)].get(short)
                    if old is not None and old != flag:
                        state_changes.append(f'{short}:{old}->{flag}')
                    last_rates[str(p)][short] = flag
                if fields:
                    label = 'RATES'
                    detail = f'role={host} window={r.get("seconds", "?")}s ' + ' '.join(fields)
                    if state_changes:
                        detail += ' CHANGES=' + ','.join(state_changes)
                    # Print every 10-second interval. Otherwise transient drops
                    # can disappear from the final timeline entirely.
            elif kind == 'pi_sample':
                check = r.get('check', '?')
                content = r.get('output')
                if content is None:
                    content = {k: v for k, v in r.items()
                               if k not in ('time', 'epoch', 'monotonic', 'kind', 'check')}
                detail = f'{check}: {str(content).replace(chr(10), " ")[:650]}'
            elif kind == 'status':
                detail = f'{r.get("topic")}: {str(r.get("value"))[:650]}'
            elif kind == 'journal':
                detail = str(r.get('line', ''))[:650]
            elif kind in ('stop', 'clock_jump', 'ros_unavailable', 'ros_error',
                          'ros_topic_error', 'ros_status_error', 'ros_discovery_error',
                          'ros_cleanup_error', 'journal_coverage', 'journal_started',
                          'journal_ended', 'journal_unavailable', 'status_unavailable'):
                detail = str({k: v for k, v in r.items()
                              if k not in ('time', 'epoch', 'monotonic', 'kind')})[:650]
            if detail is not None:
                events.append((time_key, p.name, number, mono, label, detail))
                totals[label] += 1
    return events, summary, totals, boots


def render(paths):
    events, summary, totals, boots = parse_logs(paths)
    lines = [
        'TurtleBot 4 handling test timeline',
        'IMPORTANT: Wall-clock order is provisional if the Pi clock jumped or reset.',
        'Monotonic timestamps are comparable only within the SAME host and boot.',
        'Rate=0 means the monitor received no messages; it does NOT prove sensor power loss.',
        'Laptop image subscriptions add network load; compare Pi rates to laptop rates.',
        'Thresholds: LiDAR <5 Hz LOW, RGB/depth <3 Hz LOW, zero=ZERO, ping max >=150ms spike.',
        '',
        '=== OVERVIEW ===',
    ]
    if not events:
        lines.append('No parseable events.')
    for host in sorted(summary):
        s = summary[host]
        lines.append(f'{host}: ' + ', '.join(f'{k}={v}' for k, v in sorted(s.items())))
    for host in sorted(boots):
        lines.append(f'{host}: boot IDs observed = {", ".join(sorted(boots[host]))}')
        if host == 'pi' and len(boots[host]) > 1:
            lines.append('  POSSIBLE PI REBOOT: multiple boot IDs (confirm journal/uptime).')
    if totals.get('bad_json_lines'):
        lines.append(f'WARNING: {totals["bad_json_lines"]} malformed JSONL lines ignored.')
    if not any(e[4] == 'MARKER' for e in events):
        lines.append('WARNING: No experiment markers found; ensure markers.jsonl is included.')
    if not any(e[4] == 'RATES' for e in events):
        lines.append('WARNING: No ROS rate records found; confirm ROS monitor started.')
    lines.extend(('', '=== EVENTS (provisional UTC order) ==='))
    for ts, name, number, mono, label, detail in sorted(events,
                                                       key=lambda ev: (ev[0], ev[1], ev[2])):
        lines.append(f'{ts} [{name}:{number}] mono={mono} {label}: {detail}')
    lines.extend(('', '=== INTERPRETATION NOTES ===',
                  '- Pi rates remain healthy while laptop rates/ping collapse: suspect Wi-Fi/transport.',
                  '- Both hosts lose frames while Pi uptime/boot ID remains stable: check drivers, USB and ROS.',
                  '- Pi boot ID changes with Wi-Fi loss: a reboot/power interruption is plausible.',
                  '- A missing Pi heartbeat is not proof of a reboot (logger might have died).',
                  '- Journal access may be limited; no journal errors does NOT prove none occurred.',
                  '- Dock/lift and manual markers indicate correlation, not by themselves causation.'))
    return '\n'.join(lines) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('logs', nargs='+', type=Path)
    args = p.parse_args()
    print(render(args.logs), end='')


if __name__ == '__main__':
    main()
