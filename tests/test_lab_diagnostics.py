"""Offline checks for diagnostic failure handling; no robot or ROS required."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'lab_diagnostics', Path(__file__).resolve().parents[1] / 'scripts/lab_diagnostics.py')
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)


class DiagnosticsTest(unittest.TestCase):
    def test_empty_graph_is_unknown(self):
        with patch.object(lab, 'run', return_value=(0, '')), contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertNotEqual(lab.show('graph', ['unused'], require_output=True), 0)
        self.assertIn('[UNKNOWN]', out.getvalue())

    def test_tf_missing_fails_preflight_even_with_fresh_sensors(self):
        with patch.object(lab, 'show', return_value=0), patch.object(lab, 'run', return_value=(124, 'No transform')), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(lab.preflight(), 1)

    def test_timeout_bounds_child_that_keeps_stdout_open(self):
        # The child inherits stdout; killing only its parent would leave it open.
        code = "import subprocess,sys,time; subprocess.Popen([sys.executable,'-c','import time; time.sleep(20)']); print('started',flush=True); time.sleep(20)"
        rc, output = lab.run([sys.executable, '-c', code], limit=0.2)
        self.assertEqual(rc, 124)
        self.assertIn('started', output)

    def test_evidence_includes_failures_semantics_and_bounded_log(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'asn-lab-launch.log').write_text('x' * 100000)
            with patch.object(lab, 'ROOT', root), patch.object(Path, 'home', return_value=root), patch.object(lab, 'run', return_value=(124, 'unavailable')), patch.object(lab, 'process_rows', return_value=iter([])), contextlib.redirect_stdout(io.StringIO()):
                lab.evidence()
            folder = next((root / 'results/lab_evidence').iterdir())
            self.assertIn('EXIT: 124', (folder / 'semantic_samples.txt').read_text())
            self.assertIn('EXIT: 124', (folder / 'nodes.txt').read_text())
            self.assertLess((folder / 'launch_log_tail.txt').stat().st_size, 66000)


if __name__ == '__main__':
    unittest.main()
