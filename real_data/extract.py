"""Decode a ROS 2 MCAP bag into one parquet table per topic, plus its static transforms.

Usage: python real_data/extract.py <bag_directory> [--run NAME]
Output: real_data/data/<NAME>/extracted/ (or real_data/data/extracted/ without --run), gitignored.
Topics missing from a bag give empty tables. /tf_static goes to tf_static.json (base_link -> child).
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

ap = argparse.ArgumentParser()
ap.add_argument('bag', type=Path, help='rosbag2 directory containing metadata.yaml and the .mcap file')
ap.add_argument('--run', help='run name: output goes to real_data/data/<run>/extracted')
args = ap.parse_args()
DATA = Path(__file__).resolve().parent / 'data'
OUT = (DATA / args.run if args.run else DATA) / 'extracted'
OUT.mkdir(parents=True, exist_ok=True)


def stamp_to_t(msg):
    try:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except AttributeError:
        return None


ODOM = ('/odometry/filtered', '/odometry/ekf')
TWIST = ('/dvl/raw', '/ekf/dvl_twist')
rows = {t: [] for t in ('/imu/data', '/imu/euler_deg', '/pressure/depth', *TWIST, *ODOM, '/makara/z_used',
                        '/makara/pico/armed', '/makara/pico/link_ok', '/makara/pico/esc_pwm')}
tf_static = {}

with AnyReader([args.bag]) as reader:
    conns = [c for c in reader.connections if c.topic in rows or c.topic == '/tf_static']
    for conn, t_ns, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        t_bag = t_ns * 1e-9
        topic = conn.topic
        if topic == '/tf_static':
            for tf in msg.transforms:
                if tf.header.frame_id == 'base_link':
                    tr, q = tf.transform.translation, tf.transform.rotation
                    tf_static[tf.child_frame_id] = {'t': [tr.x, tr.y, tr.z], 'q': [q.x, q.y, q.z, q.w]}
        elif topic == '/imu/data':
            rows[topic].append((t_bag, stamp_to_t(msg),
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        elif topic == '/imu/euler_deg':
            rows[topic].append((t_bag, stamp_to_t(msg), msg.vector.x, msg.vector.y, msg.vector.z))
        elif topic == '/pressure/depth':
            rows[topic].append((t_bag, msg.data))
        elif topic in TWIST:
            tw = msg.twist.twist
            cov = np.array(msg.twist.covariance)
            rows[topic].append((t_bag, stamp_to_t(msg), tw.linear.x, tw.linear.y, tw.linear.z,
                                 tw.angular.x, tw.angular.y, tw.angular.z, cov[0], cov[7], cov[14]))
        elif topic in ODOM:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            tw = msg.twist.twist
            rows[topic].append((t_bag, stamp_to_t(msg), p.x, p.y, p.z, o.x, o.y, o.z, o.w,
                                 tw.linear.x, tw.linear.y, tw.linear.z))
        elif topic in ('/makara/z_used', '/makara/pico/armed', '/makara/pico/link_ok'):
            rows[topic].append((t_bag, msg.data))
        elif topic == '/makara/pico/esc_pwm':
            rows[topic].append((t_bag, *list(msg.data)))

twist_cols = ['t_bag', 't_hdr', 'vx', 'vy', 'vz', 'avx', 'avy', 'avz', 'cov_xx', 'cov_yy', 'cov_zz']
odom_cols = ['t_bag', 't_hdr', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz']
cols = {
    '/imu/data': ['t_bag', 't_hdr', 'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'],
    '/imu/euler_deg': ['t_bag', 't_hdr', 'roll', 'pitch', 'yaw'],
    '/pressure/depth': ['t_bag', 'depth'],
    **{t: twist_cols for t in TWIST}, **{t: odom_cols for t in ODOM},
    '/makara/z_used': ['t_bag', 'z_used'],
    '/makara/pico/armed': ['t_bag', 'armed'],
    '/makara/pico/link_ok': ['t_bag', 'link_ok'],
}

for topic, data in rows.items():
    if topic == '/makara/pico/esc_pwm':
        df = pd.DataFrame(data, columns=['t_bag'] + [f'pwm{i}' for i in range(len(data[0]) - 1)]) if data else pd.DataFrame()
    else:
        df = pd.DataFrame(data, columns=cols[topic])
    df.to_parquet(OUT / (topic.strip('/').replace('/', '_') + '.parquet'))
    print(topic, df.shape)

(OUT / 'tf_static.json').write_text(json.dumps(tf_static, indent=1))
print('tf_static children:', sorted(tf_static))
print('wrote', OUT)
