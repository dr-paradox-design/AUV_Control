"""Decode a ROS 2 MCAP bag into one parquet table per topic.

Usage: python real_data/extract.py <bag_directory>
Output: real_data/data/extracted/*.parquet (gitignored)
"""
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.highlevel import AnyReader

ap = argparse.ArgumentParser()
ap.add_argument('bag', type=Path, help='rosbag2 directory containing metadata.yaml and the .mcap file')
BAG = ap.parse_args().bag
OUT = Path(__file__).resolve().parent / 'data' / 'extracted'
OUT.mkdir(parents=True, exist_ok=True)

def stamp_to_t(msg):
    try:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except AttributeError:
        return None

rows = {
    '/imu/data': [],
    '/imu/euler_deg': [],
    '/pressure/depth': [],
    '/dvl/raw': [],
    '/odometry/filtered': [],
    '/makara/z_used': [],
    '/makara/pico/armed': [],
    '/makara/pico/link_ok': [],
    '/makara/pico/esc_pwm': [],
}

with AnyReader([BAG]) as reader:
    conns = [c for c in reader.connections if c.topic in rows]
    for conn, t_ns, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        t_bag = t_ns * 1e-9
        topic = conn.topic
        if topic == '/imu/data':
            rows[topic].append((t_bag, stamp_to_t(msg),
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        elif topic == '/imu/euler_deg':
            rows[topic].append((t_bag, stamp_to_t(msg), msg.vector.x, msg.vector.y, msg.vector.z))
        elif topic == '/pressure/depth':
            rows[topic].append((t_bag, msg.data))
        elif topic == '/dvl/raw':
            tw = msg.twist.twist
            cov = np.array(msg.twist.covariance)
            rows[topic].append((t_bag, stamp_to_t(msg), tw.linear.x, tw.linear.y, tw.linear.z,
                                 tw.angular.x, tw.angular.y, tw.angular.z, cov[0], cov[7], cov[14]))
        elif topic == '/odometry/filtered':
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            tw = msg.twist.twist
            rows[topic].append((t_bag, stamp_to_t(msg), p.x, p.y, p.z, o.x, o.y, o.z, o.w,
                                 tw.linear.x, tw.linear.y, tw.linear.z))
        elif topic == '/makara/z_used':
            rows[topic].append((t_bag, msg.data))
        elif topic == '/makara/pico/armed':
            rows[topic].append((t_bag, msg.data))
        elif topic == '/makara/pico/link_ok':
            rows[topic].append((t_bag, msg.data))
        elif topic == '/makara/pico/esc_pwm':
            rows[topic].append((t_bag, *list(msg.data)))

cols = {
    '/imu/data': ['t_bag','t_hdr','qx','qy','qz','qw','wx','wy','wz','ax','ay','az'],
    '/imu/euler_deg': ['t_bag','t_hdr','roll','pitch','yaw'],
    '/pressure/depth': ['t_bag','depth'],
    '/dvl/raw': ['t_bag','t_hdr','vx','vy','vz','avx','avy','avz','cov_xx','cov_yy','cov_zz'],
    '/odometry/filtered': ['t_bag','t_hdr','x','y','z','qx','qy','qz','qw','vx','vy','vz'],
    '/makara/z_used': ['t_bag','z_used'],
    '/makara/pico/armed': ['t_bag','armed'],
    '/makara/pico/link_ok': ['t_bag','link_ok'],
}

for topic, data in rows.items():
    if topic == '/makara/pico/esc_pwm':
        if data:
            n = len(data[0]) - 1
            df = pd.DataFrame(data, columns=['t_bag'] + [f'pwm{i}' for i in range(n)])
        else:
            df = pd.DataFrame()
    else:
        df = pd.DataFrame(data, columns=cols[topic])
    fname = topic.strip('/').replace('/', '_') + '.parquet'
    df.to_parquet(OUT / fname)
    print(topic, df.shape)

print("done")
