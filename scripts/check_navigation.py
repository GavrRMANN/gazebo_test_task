import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud2
from std_msgs.msg import String


def obstacle_contact(first, second):
    robot = 'autonomy_robot::'
    if first.startswith(robot) and not second.startswith(robot):
        return not second.startswith('ground_plane::')
    if second.startswith(robot) and not first.startswith(robot):
        return not first.startswith('ground_plane::')
    return False


class ContactMonitor:
    def __init__(self):
        self.process = None
        self.thread = None
        self.pairs = 0
        self.collisions = set()
        self.stopped = threading.Event()

    def start(self):
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        while not self.stopped.is_set():
            self.process = subprocess.Popen(
                ['bash', '-c', 'gz topic -e /gazebo/autonomy_world/physics/contacts '
                 "| grep --line-buffered -E 'collision[12]:'"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                start_new_session=True,
            )
            if self.stopped.is_set():
                self.terminate()
                return
            first = None
            for line in self.process.stdout:
                value = line.split('"')[1]
                if 'collision1:' in line:
                    first = value
                elif first is not None:
                    if 'autonomy_robot::' in first or 'autonomy_robot::' in value:
                        self.pairs += 1
                    if obstacle_contact(first, value):
                        self.collisions.add((first, value))
                    first = None
            if self.stopped.wait(0.5):
                return

    def terminate(self):
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.process.wait(timeout=5)

    def close(self):
        self.stopped.set()
        self.terminate()
        if self.thread is not None:
            self.thread.join(timeout=2)


def rectangle(x, y, yaw, half_x, half_y):
    return [(x + math.cos(yaw) * u - math.sin(yaw) * v,
             y + math.sin(yaw) * u + math.cos(yaw) * v)
            for u, v in ((-half_x, -half_y), (half_x, -half_y),
                         (half_x, half_y), (-half_x, half_y))]


def edges(polygon):
    return list(zip(polygon, polygon[1:] + polygon[:1]))


def point_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    fraction = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy)
                         / (dx * dx + dy * dy)))
    return math.hypot(point[0] - a[0] - fraction * dx,
                      point[1] - a[1] - fraction * dy)


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def inside(point, polygon):
    values = [cross(a, b, point) for a, b in edges(polygon)]
    return all(v >= 0 for v in values) or all(v <= 0 for v in values)


def polygon_clearance(first, second):
    if inside(first[0], second) or inside(second[0], first):
        return -0.001
    for a, b in edges(first):
        for c, d in edges(second):
            if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
                return -0.001
    return min([point_distance(p, a, b) for p in first for a, b in edges(second)]
               + [point_distance(p, a, b) for p in second for a, b in edges(first)])


def obstacles(path):
    result = []
    for model in ET.parse(path).findall('.//world/model'):
        pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        for collision in model.findall('link/collision'):
            geometry = collision.find('geometry')
            box = geometry.find('box')
            cylinder = geometry.find('cylinder')
            if box is not None:
                size = [float(v) for v in box.findtext('size').split()]
                result.append(('box', rectangle(pose[0], pose[1], pose[5], size[0] / 2, size[1] / 2)))
            elif cylinder is not None:
                result.append(('circle', (pose[0], pose[1], float(cylinder.findtext('radius')))))
    return result


def clearance(pose, barriers):
    x, y, yaw = pose
    footprint = rectangle(x, y, yaw, 0.27, 0.23)
    distances = []
    for kind, obstacle in barriers:
        if kind == 'box':
            distances.append(polygon_clearance(footprint, obstacle))
        else:
            center = obstacle[:2]
            distance = min(point_distance(center, a, b) for a, b in edges(footprint))
            distances.append(-obstacle[2] if inside(center, footprint) else distance - obstacle[2])
    return min(distances)


def trial(number, args, barriers):
    node = rclpy.create_node(f'navigation_validation_{number}')
    samples = 0
    minimum = math.inf
    position = None
    result = None
    started_mission = False
    counts = {'lidar': 0, 'depth': 0, 'gps': 0}
    contacts = ContactMonitor()

    def odometry(message):
        nonlocal samples, minimum, position
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        position = (p.x, p.y, yaw)
        minimum = min(minimum, clearance(position, barriers))
        samples += 1
        contacts.start()

    def status(message):
        nonlocal result, started_mission
        data = json.loads(message.data)
        if data['state'] in ('WAITING', 'NAVIGATING'):
            started_mission = True
        if started_mission and data['state'] in ('SUCCEEDED', 'FAILED'):
            result = data

    node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
    node.create_subscription(String, '/mission/status', status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    subscriptions = []
    for key, topic, message_type in (
        ('lidar', '/scan', LaserScan), ('depth', '/camera/depth_camera/points', PointCloud2),
        ('gps', '/gps/fix', NavSatFix),
    ):
        subscriptions.append(node.create_subscription(
            message_type, topic,
            lambda message, key=key: counts.__setitem__(key, counts[key] + 1),
            qos_profile_sensor_data,
        ))
    log_path = Path('/tmp') / f'navigation_validation_{number}.log'
    command = ['bash', '/workspace/scripts/run_demo.sh']
    if args.headless:
        command.extend(['gui:=false', 'rviz:=false'])
    start = last_progress = time.monotonic()
    recording = bag = None
    video_duration = None
    last_gui_check = start
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while result is None and time.monotonic() - start < args.timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                if args.video and recording is None and time.monotonic() - last_gui_check >= 1:
                    last_gui_check = time.monotonic()
                    if all(subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL).returncode == 0
                           for name in ('gzclient', 'rviz2')):
                        Path(args.video).parent.mkdir(parents=True, exist_ok=True)
                        recording = subprocess.Popen([
                            'ffmpeg', '-n', '-f', 'x11grab', '-framerate', '15',
                            '-video_size', '1600x900', '-i', ':1', '-t', '179',
                            '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '25',
                            '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                            args.video], stdout=log, stderr=log)
                        if args.bag:
                            Path(args.bag).parent.mkdir(parents=True, exist_ok=True)
                            bag = subprocess.Popen([
                                'ros2', 'bag', 'record', '--use-sim-time', '-o', args.bag,
                                '/scan', '/gps/fix', '/odom', '/tf', '/tf_static',
                                '/cmd_vel_safe', '/mission/status',
                                '/camera/depth_camera/depth/image_raw'], stdout=log, stderr=log)
                if process.poll() is not None:
                    result = {'state': 'FAILED', 'reason': f'Launch exited with code {process.returncode}'}
                    break
                if time.monotonic() - last_progress >= 10:
                    print(json.dumps({'run': number, 'position': position,
                                      'clearance': minimum if samples else None}), flush=True)
                    last_progress = time.monotonic()
            complete_video = not args.video or (recording is not None and recording.poll() is None)
            if recording is not None:
                time.sleep(3)
                recording.send_signal(signal.SIGINT)
                recording.wait(timeout=20)
                probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                                        '-of', 'json', args.video], capture_output=True, text=True)
                if probe.returncode == 0:
                    video_duration = float(json.loads(probe.stdout)['format']['duration'])
                complete_video = complete_video and video_duration is not None and 60 <= video_duration <= 180
            if bag is not None:
                bag.send_signal(signal.SIGINT)
                bag.wait(timeout=20)
            gui = all(subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL).returncode == 0
                      for name in ('gzclient', 'rviz2')) if not args.headless else None
            model = subprocess.run(['gz', 'model', '-m', 'autonomy_robot', '-p'],
                                   capture_output=True, text=True, timeout=10)
            ground_truth = [float(v) for v in model.stdout.split()] if model.returncode == 0 else []
            contacts.close()
            passed = bool(result and result['state'] == 'SUCCEEDED' and samples > 100
                          and minimum > 0 and all(v > 1 for v in counts.values())
                          and contacts.pairs > 100 and not contacts.collisions
                          and complete_video
                          and (not args.bag or Path(args.bag, 'metadata.yaml').is_file())
                          and ground_truth and math.hypot(ground_truth[0] - 4, ground_truth[1] - 3) < 0.35
                          and (args.headless or gui))
            return {'run': number, 'passed': passed, 'result': result,
                    'wall_seconds': round(time.monotonic() - start, 2),
                    'odom_samples': samples, 'min_clearance_m': minimum if samples else None,
                    'sensor_messages': counts, 'ground_truth': ground_truth,
                    'gazebo_contact_pairs': contacts.pairs,
                    'obstacle_contacts': sorted(contacts.collisions),
                    'video': args.video, 'video_seconds': video_duration, 'bag': args.bag,
                    'gui_running': gui, 'log': str(log_path)}
        finally:
            for capture in (recording, bag):
                if capture is not None and capture.poll() is None:
                    capture.send_signal(signal.SIGINT)
                    capture.wait(timeout=20)
            contacts.close()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--report', default='/workspace/.ai/maze_validation.json')
    parser.add_argument('--video')
    parser.add_argument('--bag')
    args = parser.parse_args()
    if args.video and (args.runs != 1 or args.headless):
        parser.error('--video requires one GUI run (--runs 1)')
    if args.bag and not args.video:
        parser.error('--bag requires --video')
    if args.video and Path(args.video).exists():
        parser.error('Video output already exists')
    if args.bag and Path(args.bag).exists():
        parser.error('Bag output already exists')
    barriers = obstacles('/workspace/src/autonomy_gazebo/worlds/test_world.world')
    rclpy.init()
    results = []
    try:
        for number in range(1, args.runs + 1):
            result = trial(number, args, barriers)
            results.append(result)
            Path(args.report).write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(result), flush=True)
            if not result['passed']:
                raise SystemExit(1)
            time.sleep(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
