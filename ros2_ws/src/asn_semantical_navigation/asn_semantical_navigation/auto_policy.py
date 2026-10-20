"""Pure helper for determining whether autonomous replanning is safe."""

import re


def parse_navigation_summary(message):
    """Parse one Step-2 summary; never infer success from goal acceptance."""
    succeeded = (message.startswith('NAV2_SUCCEEDED |') and
                 'pose_check=PASS' in message)
    match = re.search(r'actual coverage gain=([0-9.]+)m2', message)
    gain = float(match.group(1)) if match else None
    return succeeded, gain
