from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch import LaunchContext
import pytest


def test_launch_lock_lifetime(tmp_path, monkeypatch):
    launch_file = Path(__file__).resolve().parents[1] / 'launch/hardware/navigation_hardware.launch.py'
    spec = spec_from_file_location('hardware_launch', launch_file)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    # Model a clean host even when tests run beside a hardware launch.
    monkeypatch.setattr(Path, 'glob', lambda *args: [])
    context = LaunchContext()
    context.launch_configurations['map'] = str(launch_file)
    module.check_map(context)
    with pytest.raises(RuntimeError, match='holds the lock'):
        module.check_map(context)
    context.locals.asn_hardware_navigation_lock.close()
    module.check_map(context)
    context.locals.asn_hardware_navigation_lock.close()
