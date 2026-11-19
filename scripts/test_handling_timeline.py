#!/usr/bin/env python3
"""Offline regression tests for handling monitor timeline. No ROS/network required."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent


def rec(time, kind, **args):
    return dict(time=time, kind=kind, monotonic=123.0, **args)


class TimelineRegressionTests(unittest.TestCase):
    def render(self, logs):
        with tempfile.TemporaryDirectory() as folder:
            paths = []
            for filename, records in logs.items():
                p = Path(folder) / filename
                p.write_text('\n'.join(json.dumps(r) if isinstance(r, dict) else r
                                       for r in records) + '\n')
                paths.append(str(p))
            return subprocess.check_output(
                [sys.executable, str(ROOT / 'handling_timeline.py'), *paths], text=True)

    def test_sensor_zero_and_recovery_are_not_hidden(self):
        time = '2026-11-17T23:40:00.000+00:00'
        topics = ('/scan', '/oakd/rgb/image_raw/compressed',
                  '/oakd/stereo/image_raw/compressedDepth')
        sample = lambda n: rec(time, 'rates', role='pi', seconds=10,
                               hz=dict(zip(topics, n)))
        text = self.render({'pi-test.jsonl': [
            rec(time, 'start', role='pi', boot_id='boot-a'),
            sample((7.5, 5.0, 5.0)), sample((0, 0, 0)),
            sample((7.4, 4.9, 5.1)),
        ]})
        self.assertIn('scan=0.00Hz(ZERO)', text)
        self.assertIn('rgb=0.00Hz(ZERO)', text)
        self.assertIn('depth=0.00Hz(ZERO)', text)
        self.assertIn('scan:OK->ZERO', text)
        self.assertIn('scan:ZERO->OK', text)
        self.assertIn('boot-a', text)

    def test_partial_ping_loss_and_latency_are_reported(self):
        time = '2026-11-17T23:40:00+00:00'
        def network(ping, link='Connected to e4:5f:01:bd:04:78\n\tsignal: -62 dBm'):
            return rec(time, 'network', role='laptop',
                       ping={'rc': 0, 'output': ping},
                       link={'rc': 0, 'output': link})
        text = self.render({'laptop.jsonl': [
            network('3 packets transmitted, 3 received, 0% packet loss, time 2000ms\n'
                    'rtt min/avg/max/mdev = 4/20/40/10 ms'),
            network('3 packets transmitted, 2 received, 33.3333% packet loss, time 2000ms\n'
                    'rtt min/avg/max/mdev = 4/30/200/12 ms'),
            network('3 packets transmitted, 0 received, 100% packet loss, time 2000ms',
                    'Not connected.'),
        ]})
        self.assertIn('ping=PARTIAL loss=33.3%', text)
        self.assertIn('latency_spike=True', text)
        self.assertIn('ping=DOWN loss=100.0%', text)
        self.assertIn('network_PARTIAL=1', text)
        self.assertIn('network_DOWN=1', text)

    def test_ping_summary_with_kernel_errors(self):
        spec = importlib.util.spec_from_file_location('handling_timeline', ROOT / 'handling_timeline.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        info = mod.ping_info('3 packets transmitted, 0 received, +1 errors, 100% packet loss, time 2000ms')
        self.assertEqual(info['loss'], 100.0)
        self.assertEqual(info['received'], 0)

    def test_markers_and_pi_boot_ids(self):
        time = '2026-11-17T23:40:00+00:00'
        text = self.render({
            'pi-a.jsonl': [rec(time, 'start', role='pi', boot_id='old-boot')],
            'pi-b.jsonl': [rec(time, 'start', role='pi', boot_id='new-boot')],
            'markers.jsonl': [rec(time, 'marker', test='B', note='START undocked same table')],
        })
        self.assertIn('POSSIBLE PI REBOOT', text)
        self.assertIn('trial=B START undocked same table', text)

    def test_markers_written_separately_with_atomic_append(self):
        spec = importlib.util.spec_from_file_location('handling_monitor', ROOT / 'handling_monitor.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'markers.jsonl'
            module.record_marker(str(p), 'A', 'START docked')
            module.record_marker(str(p), 'A', 'END docked')
            records = [json.loads(line) for line in p.read_text().splitlines()]
            self.assertEqual([r['note'] for r in records], ['START docked', 'END docked'])
            self.assertEqual(len(records), 2)

    def test_lidar_device_check_includes_case_sensitive_path(self):
        spec = importlib.util.spec_from_file_location('handling_monitor', ROOT / 'handling_monitor.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertIn('rplidar_present', module.lidar_devices())


if __name__ == '__main__':
    unittest.main(verbosity=2)
