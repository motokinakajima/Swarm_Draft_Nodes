#!/usr/bin/env python3
"""
Extract swarm state from a ros2 bag into a JSON timeline for bag_visualizer's
browser player.

Robots are keyed by their topic namespace prefix, per rosbag_record.txt:
  root robot  -> no prefix        (/mavros/..., /peer_detection, /debug/...)
  robot1      -> /robot1 prefix   (/robot1/mavros/..., /robot1/peer_detection, ...)

Usage:
  python3 extract_bag.py <path-to-bag> -o data.json [--rate 10] \
      [--temp-field ../fake_tempratures/wilsons_landing_smaller_multimodal.mat]
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import utm
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

ROBOTS = {
    'root': '',
    'robot1': '/robot1',
}

FORCE_TOPICS = {
    'avoid': 'debug/force_avoidance',
    'quark': 'debug/force_quark',
    'dir': 'debug/force_directional',
    'net': 'debug/force_net',
}


def candidate_topics(prefix, suffix):
    """rosbag_record.txt has a typo (missing leading slash) on one topic,
    and namespacing can go either way, so try the sane variants."""
    bare = suffix.lstrip('/')
    cands = {f'{prefix}/{bare}', f'{prefix}/{suffix}'.replace('//', '/')}
    if prefix == '':
        cands.add(f'/{bare}')
        cands.add(bare)
    return cands


def build_topic_map(reader):
    available = {c.topic for c in reader.connections}
    topic_map = {}
    for robot, prefix in ROBOTS.items():
        entry = {}
        entry['navsat'] = _pick(available, candidate_topics(prefix, 'mavros/global_position/global'))
        entry['heading'] = _pick(available, candidate_topics(prefix, 'mavros/global_position/compass_hdg'))
        entry['velocity'] = _pick(available, candidate_topics(prefix, 'velocity'))
        entry['cmd_vel'] = _pick(available, candidate_topics(prefix, 'mavros/setpoint_velocity/cmd_vel'))
        entry['peers'] = _pick(available, candidate_topics(prefix, 'peer_detection'))
        entry['forces'] = {}
        for key, suffix in FORCE_TOPICS.items():
            entry['forces'][key] = _pick(available, candidate_topics(prefix, suffix))
        topic_map[robot] = entry
    return topic_map


def _pick(available, candidates):
    for c in candidates:
        if c in available:
            return c
    return None


def rotate_body_to_map(x_body, y_body, heading_deg):
    yaw = math.radians(heading_deg)
    east = x_body * math.sin(yaw) - y_body * math.cos(yaw)
    north = x_body * math.cos(yaw) + y_body * math.sin(yaw)
    return east, north


def extract(bag_path, out_path, rate_hz, temp_field_path, ros_distro):
    bag_path = Path(bag_path)
    # Some bags (e.g. recorded without --include-hidden-topics / with a
    # storage config that doesn't embed full msg definitions) don't carry
    # their own type definitions, so AnyReader can't self-derive types and
    # needs an explicit fallback typestore for the standard message types
    # this project uses (NavSatFix, TwistStamped, PolygonStamped, etc).
    typestore = get_typestore(getattr(Stores, ros_distro))
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        topic_map = build_topic_map(reader)

        for robot, entry in topic_map.items():
            missing = [k for k, v in entry.items() if k not in ('forces',) and v is None]
            missing += [k for k, v in entry['forces'].items() if v is None]
            if missing:
                print(f"[warn] robot '{robot}': no topic found for {missing}")

        topic_to_target = {}
        for robot, entry in topic_map.items():
            for key in ('navsat', 'heading', 'velocity', 'cmd_vel', 'peers'):
                if entry[key]:
                    topic_to_target[entry[key]] = (robot, key)
            for fkey, topic in entry['forces'].items():
                if topic:
                    topic_to_target[topic] = (robot, 'force', fkey)

        connections = [c for c in reader.connections if c.topic in topic_to_target]
        if not connections:
            raise SystemExit('No matching topics found in this bag. Check rosbag_record.txt against the bag contents.')

        events = []
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            target = topic_to_target[connection.topic]
            events.append((timestamp, target, msg))

        events.sort(key=lambda e: e[0])
        t0 = events[0][0]
        t_end = events[-1][0]

        utm_zone = {}

        def to_local(lat, lon):
            if 'num' not in utm_zone:
                e, n, zn, zl = utm.from_latlon(lat, lon)
                utm_zone['num'], utm_zone['letter'] = zn, zl
                utm_zone['origin_e'], utm_zone['origin_n'] = e, n
                utm_zone['origin_lat'], utm_zone['origin_lon'] = lat, lon
                return 0.0, 0.0
            e, n, _, _ = utm.from_latlon(lat, lon, force_zone_number=utm_zone['num'], force_zone_letter=utm_zone['letter'])
            return e - utm_zone['origin_e'], n - utm_zone['origin_n']

        state = {
            robot: {
                'pos': None, 'heading_deg': None,
                'peers_body': [], 'forces': {'avoid': [0.0, 0.0], 'quark': [0.0, 0.0], 'dir': [0.0, 0.0], 'net': [0.0, 0.0]},
                'velocity': [0.0, 0.0, 0.0], 'cmd_vel': [0.0, 0.0, 0.0],
            }
            for robot in ROBOTS
        }

        dt = 1.0 / rate_hz
        frames = []
        next_sample_t = t0
        ei = 0
        n_events = len(events)

        while next_sample_t <= t_end:
            while ei < n_events and events[ei][0] <= next_sample_t:
                ts, target, msg = events[ei]
                robot = target[0]
                s = state[robot]
                if target[1] == 'navsat':
                    x, y = to_local(msg.latitude, msg.longitude)
                    s['pos'] = [x, y]
                elif target[1] == 'heading':
                    s['heading_deg'] = float(msg.data)
                elif target[1] == 'peers':
                    s['peers_body'] = [[p.x, p.y] for p in msg.polygon.points]
                elif target[1] == 'velocity':
                    s['velocity'] = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
                elif target[1] == 'cmd_vel':
                    s['cmd_vel'] = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]
                elif target[1] == 'force':
                    fkey = target[2]
                    s['forces'][fkey] = [msg.vector.x, msg.vector.y]
                ei += 1

            frame = {'t': round((next_sample_t - t0) / 1e9, 3), 'robots': {}}
            for robot, s in state.items():
                if s['pos'] is None:
                    frame['robots'][robot] = None
                    continue
                heading = s['heading_deg'] if s['heading_deg'] is not None else 0.0
                peers_map = []
                for px, py in s['peers_body']:
                    de, dn = rotate_body_to_map(px, py, heading)
                    peers_map.append([s['pos'][0] + de, s['pos'][1] + dn])
                forces_map = {}
                for fkey, (fx, fy) in s['forces'].items():
                    de, dn = rotate_body_to_map(fx, fy, heading)
                    forces_map[fkey] = [de, dn]
                frame['robots'][robot] = {
                    'pos': s['pos'],
                    'heading_deg': heading,
                    'peers': peers_map,
                    'forces': forces_map,
                    'velocity': s['velocity'],
                    'cmd_vel': s['cmd_vel'],
                }
            frames.append(frame)
            next_sample_t += dt * 1e9

        output = {
            'meta': {
                'origin_lat': utm_zone.get('origin_lat'),
                'origin_lon': utm_zone.get('origin_lon'),
                'dt': dt,
                'duration': frames[-1]['t'] if frames else 0.0,
                'robots': list(ROBOTS.keys()),
                'topic_map': topic_map,
            },
            'frames': frames,
        }

        if temp_field_path:
            output['field'] = extract_temp_field(temp_field_path, utm_zone)

        Path(out_path).write_text(json.dumps(output))
        print(f"Wrote {len(frames)} frames ({output['meta']['duration']:.1f}s) to {out_path}")


def extract_temp_field(mat_path, utm_zone):
    import scipy.io
    data = scipy.io.loadmat(mat_path)
    lat_mesh = data['latMesh']
    lon_mesh = data['lonMesh']
    z_mean = data['zMean']

    if 'num' not in utm_zone:
        raise SystemExit('Cannot place temperature field: bag had no navsat fixes to establish an origin.')

    rows, cols = lat_mesh.shape
    xs = np.zeros((rows, cols))
    ys = np.zeros((rows, cols))
    for r in range(rows):
        for c in range(cols):
            e, n, _, _ = utm.from_latlon(
                float(lat_mesh[r, c]), float(lon_mesh[r, c]),
                force_zone_number=utm_zone['num'], force_zone_letter=utm_zone['letter']
            )
            xs[r, c] = e - utm_zone['origin_e']
            ys[r, c] = n - utm_zone['origin_n']

    return {
        'x': xs.tolist(),
        'y': ys.tolist(),
        'z': z_mean.tolist(),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag_path', help='Path to a ros2 bag directory (containing metadata.yaml)')
    ap.add_argument('-o', '--out', default='data.json', help='Output JSON path')
    ap.add_argument('--rate', type=float, default=10.0, help='Playback sample rate in Hz (default 10, matches swarm_captain control rate)')
    ap.add_argument('--temp-field', default=None, help='Optional .mat file (e.g. fake_tempratures/*.mat) to overlay as a background heatmap')
    ap.add_argument('--ros-distro', default='LATEST',
                     choices=[s.name for s in Stores],
                     help='Fallback typestore to use if the bag has no embedded type definitions (default: LATEST)')
    args = ap.parse_args()
    extract(args.bag_path, args.out, args.rate, args.temp_field, args.ros_distro)


if __name__ == '__main__':
    main()
