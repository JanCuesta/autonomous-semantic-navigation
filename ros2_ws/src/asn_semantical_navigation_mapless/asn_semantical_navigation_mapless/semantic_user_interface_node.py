"""Interactive terminal front end for the existing semantic mission manager."""
import json
import select
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .search_commands import parse_target

TERMINAL = {'IDLE', 'SUCCEEDED', 'FAILED', 'CANCELLED'}
DESCRIPTIONS = {
    'PREPARING': 'Checking YOLO, TF and Nav2...',
    'SEARCHING': 'Looking for {target}...',
    'STOPPING_SEARCH': 'Stopping exploration...',
    'LOCALIZING': 'Estimating object position...',
    'WAITING_APPROACH': 'Computing safe approach...',
    'APPROACHING': 'Navigating toward {target}...',
    'VERIFYING': 'Checking final position...',
    'CANCELLING': 'Waiting for navigation to stop...',
}


def confirm(prompt):
    while True:
        answer = input(prompt).strip().lower()
        if answer in ('yes', 'y'):
            return True
        if answer in ('no', 'n'):
            return False
        print('Please type yes or no.')


class TerminalUI(Node):
    def __init__(self):
        super().__init__('semantic_user_interface_node')
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.lock = threading.Lock()
        self.target = ''
        self.status = None
        self.command_pub = self.create_publisher(
            String, '/semantic/search/command', 10)
        self.create_subscription(String, '/semantic/search/target_class',
                                 self.on_target, latched)
        self.create_subscription(String, '/semantic/mission/status_json',
                                 self.on_status, latched)
        self.start_client = self.create_client(
            Trigger, '/semantic/mission/start')
        self.cancel_client = self.create_client(
            Trigger, '/semantic/mission/cancel')

    def on_target(self, msg):
        with self.lock:
            self.target = msg.data.strip()

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('state'), str):
                return
        except (TypeError, ValueError):
            return
        with self.lock:
            self.status = data

    def snapshot(self):
        with self.lock:
            return self.target, (dict(self.status) if self.status else None)

    def ready(self):
        _, status = self.snapshot()
        return (status is not None and self.start_client.service_is_ready()
                and self.cancel_client.service_is_ready()
                and self.command_pub.get_subscription_count() > 0)

    def wait_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if condition():
                return True
            time.sleep(0.05)
        return False

    def call(self, client, timeout=10):
        if not client.service_is_ready():
            raise RuntimeError('Mission service unavailable')
        future = client.call_async(Trigger.Request())
        if not self.wait_until(future.done, timeout):
            raise RuntimeError('Service timed out; inspect mission status')
        response = future.result()
        if response is None:
            raise RuntimeError('Empty service response')
        return response


def monitor(node, mission_id, target):
    last = None
    print("Type 'cancel' + Enter during the mission to request cancellation.")
    watch_keyboard = sys.stdin.isatty()
    while rclpy.ok():
        _, status = node.snapshot()
        if status and status.get('mission_id') == mission_id:
            state = status['state']
            if state != last:
                last = state
                if state == 'SUCCEEDED':
                    print('\n✓ MISSION SUCCEEDED')
                elif state in ('FAILED', 'CANCELLED'):
                    print(f"\n[{state}] {status.get('reason', '')}")
                else:
                    print(f"[{state}] " + DESCRIPTIONS.get(state, state).format(target=target))
            if state in TERMINAL - {'IDLE'}:
                return
        if watch_keyboard:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                line = sys.stdin.readline()
                if not line:
                    watch_keyboard = False
                elif line.strip().lower() == 'cancel':
                    try:
                        result = node.call(node.cancel_client)
                        print('Cancellation requested; awaiting stop.' if result.success
                              else f'Cancellation rejected: {result.message}')
                    except RuntimeError as exc:
                        print(f'Cancellation unconfirmed: {exc}')
                elif line.strip():
                    print("Enter 'cancel' to stop the mission.")
        else:
            time.sleep(0.1)


def interactive(node):
    print('\n[SEMANTIC NAVIGATION]')
    print('Connecting to Semantic Mission Manager...')
    if not node.wait_until(node.ready, 20):
        print('ROS interfaces unavailable. Start semantical_navigation.launch.py first.')
        return
    print('Connected to Semantic Mission Manager.')
    while rclpy.ok():
        _, status = node.snapshot()
        if status['state'] not in TERMINAL:
            print(f"An existing mission is active: {status['state']}. Stop it first.")
            return
        request = input('\nWhat object would you like me to find?\n> ').strip()
        if request.lower() in ('quit', 'exit'):
            return
        try:
            target = parse_target(request)
        except ValueError as exc:
            print(f'Invalid request: {exc}')
            continue
        print('Sending search request...')
        node.command_pub.publish(String(data=request))
        # Validate the selected target at both the parser and the manager.
        if not node.wait_until(
                lambda: (lambda t, s: t == target and s is not None
                         and s.get('target_class') == target)(*node.snapshot()), 8):
            print('Target was not confirmed. Check the search-command node.')
            continue
        print(f'✓ Object class recognized: {target}')
        print('✓ Target registered by Mission Manager.')
        if not confirm(f'\nTarget: {target.upper()}\n'
                       'Start autonomous search and navigation? (yes/no)\n> '):
            print('Mission not started.')
            continue
        _, before = node.snapshot()
        if before['state'] not in TERMINAL:
            print('Mission already active; start cancelled.')
            return
        print('\nRequesting mission start...')
        try:
            result = node.call(node.start_client)
        except RuntimeError as exc:
            print(f'Start unconfirmed: {exc}')
            return  # Never auto-retry a start with an unknown result.
        if not result.success:
            print(f'Mission rejected: {result.message}')
            continue
        print('✓ Mission accepted (preparation started).')
        # Supervisor increments mission_id exactly once for each accepted start.
        monitor(node, int(before.get('mission_id', 0)) + 1, target)
        if not confirm('\nSearch for another object? (yes/no)\n> '):
            return


def main(args=None):
    rclpy.init(args=args)
    node = TerminalUI()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        interactive(node)
    except (KeyboardInterrupt, EOFError):
        print('\nInterface closed; any active mission is NOT cancelled automatically.')
        print("Call /semantic/mission/cancel to cancel it if needed.")
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
