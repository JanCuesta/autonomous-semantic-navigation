"""Summarize NAV-001 velocity-pipeline rosbag using receipt timestamps."""

import csv
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    '/cmd_vel_nav',
    '/cmd_vel_smoothed',
    '/diff_drive_controller/cmd_vel',
    '/odom',
}


def main(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {item.name: get_message(item.type)
             for item in reader.get_all_topics_and_types()
             if item.name in TOPICS}
    rows = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == '/odom':
            vx, wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        else:
            vx, wz = msg.twist.linear.x, msg.twist.angular.z
        rows.append((stamp_ns / 1e9, topic, vx, wz))
    if not rows:
        raise RuntimeError('No velocity/odometry messages found')
    for topic in sorted(TOPICS):
        topic_rows = [r for r in rows if r[1] == topic]
        if topic_rows:
            print(f'{topic}: first={topic_rows[0][0]:.3f} '
                  f'last={topic_rows[-1][0]:.3f}')
    command_end = max(r[0] for r in rows if r[1] == '/cmd_vel_nav')
    rows = [row for row in rows if row[0] >= command_end - 20.0]
    with open(uri + '/velocity_last20s.csv', 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['receipt_s', 'topic', 'linear_x', 'angular_z'])
        writer.writerows(rows)
    for topic in sorted(TOPICS):
        values = [r[2] for r in rows if r[1] == topic]
        if values:
            print(f'{topic}: n={len(values)} min={min(values):.5f} '
                  f'max={max(values):.5f} last={values[-1]:.5f}')


if __name__ == '__main__':
    main(sys.argv[1])
