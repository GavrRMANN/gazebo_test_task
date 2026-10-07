import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan, NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from gps_coordinates import GpsProjection


def load_mission(path):
    with Path(path).open() as stream:
        config = yaml.safe_load(stream)
    reference = config['gps_reference']
    projection = GpsProjection(**reference)
    waypoints = config['waypoints']
    if not waypoints:
        raise ValueError('The mission must contain at least one waypoint')
    goals = []
    for waypoint in waypoints:
        x, y = projection.to_world(waypoint['latitude'], waypoint['longitude'])
        yaw = float(waypoint.get('yaw', 0.0))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('Waypoint coordinates must be finite')
        if abs(x) > 4.5 or abs(y) > 3.5:
            raise ValueError('Waypoint is outside the demo arena')
        goals.append((x, y, yaw))
    return config, projection, goals


class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        self.declare_parameter('waypoints_file', '')
        self.config, self.projection, self.goals = load_mission(
            self.get_parameter('waypoints_file').value
        )
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.sensors = {}
        self.gps = None
        self.footprint_stamp = None
        for topic, message_type, key in (
            ('/scan', LaserScan, 'lidar'),
            ('/camera/depth_camera/depth/camera_info', CameraInfo, 'depth'),
            ('/odom', Odometry, 'odom'),
        ):
            self.create_subscription(
                message_type, topic,
                lambda message, key=key: self.observe(key, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(NavSatFix, '/gps/fix', self.observe_gps,
                                 qos_profile_sensor_data)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 self.observe_footprint, 10)
        self.status_pub = self.create_publisher(
            String, '/mission/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.waypoints_pub = self.create_publisher(
            MarkerArray, '/mission/waypoints',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.gps_pub = self.create_publisher(PoseStamped, '/mission/gps_pose', 10)
        markers = MarkerArray()
        for index, (x, y, _) in enumerate(self.goals):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id = 'waypoints', index
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = x, y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.g, marker.color.a = 1.0, 1.0
            markers.markers.append(marker)
        self.waypoints_pub.publish(markers)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('amcl', 'bt_navigator', 'controller_server',
                         'velocity_smoother', 'collision_monitor')
        }
        self.active = set()
        self.pending_states = set()
        self.goal_handle = None
        self.index = 0
        self.state = 'WAITING'
        self.started = time.monotonic()
        self.goal_started = None
        self.last_state_poll = 0.0
        self.last_log = 0.0
        self.ready_since = None
        self.publish_status('WAITING')
        self.create_timer(0.2, self.tick)

    def observe(self, key, message):
        stamp = message.header.stamp
        self.sensors[key] = stamp.sec + stamp.nanosec / 1e9

    def observe_gps(self, message):
        if message.status.status >= 0 and all(
            math.isfinite(v) for v in
            (message.latitude, message.longitude, message.altitude)
        ):
            self.gps = message
            self.observe('gps', message)

    def observe_footprint(self, message):
        self.footprint_stamp = (
            message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        )

    def robot_pose(self):
        transform = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        position = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y ** 2 + rotation.z ** 2),
        )
        return position.x, position.y, yaw, transform.header.stamp

    def gps_base(self, yaw):
        x, y = self.projection.to_world(
            self.gps.latitude, self.gps.longitude, self.gps.altitude
        )
        return x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw)

    def publish_status(self, state, reason=''):
        self.state = state
        data = {'state': state, 'waypoint': min(self.index + 1, len(self.goals)),
                'total': len(self.goals), 'reason': reason}
        message = String(data=json.dumps(data))
        self.status_pub.publish(message)
        self.get_logger().info(message.data)

    def fail(self, reason):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.stop_pub.publish(Twist())
        self.publish_status('FAILED', reason)

    def poll_lifecycle(self):
        for name, client in self.lifecycle.items():
            if name in self.pending_states or not client.service_is_ready():
                continue
            self.pending_states.add(name)
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda result, name=name: self.node_state(name, result))

    def node_state(self, name, future):
        self.pending_states.discard(name)
        try:
            if future.result().current_state.id == 3:
                self.active.add(name)
            else:
                self.active.discard(name)
        except Exception:
            self.active.discard(name)

    def tick(self):
        if self.state == 'FAILED':
            self.stop_pub.publish(Twist())
            return
        if self.state == 'SUCCEEDED':
            return
        wall_time = time.monotonic()
        now = self.get_clock().now().nanoseconds / 1e9
        missing = [key for key in ('lidar', 'depth', 'gps', 'odom')
                   if key not in self.sensors or not -0.5 <= now - self.sensors[key] <= 2.0]
        try:
            pose = self.robot_pose()
            pose_stamp = pose[3].sec + pose[3].nanosec / 1e9
            transform_ready = -1.5 <= now - pose_stamp <= 2.0
        except TransformException:
            pose, transform_ready = None, False
            pose_stamp = None
        footprint_ready = (self.footprint_stamp is not None
                           and -0.5 <= now - self.footprint_stamp <= 2.0)
        if self.gps is not None and pose is not None:
            gps_x, gps_y = self.gps_base(pose[2])
            gps_pose = PoseStamped()
            gps_pose.header.frame_id = 'map'
            gps_pose.header.stamp = self.gps.header.stamp
            gps_pose.pose.position.x, gps_pose.pose.position.y = gps_x, gps_y
            gps_pose.pose.orientation.w = 1.0
            self.gps_pub.publish(gps_pose)
        if self.state == 'WAITING':
            if wall_time - self.last_state_poll > 1.0:
                self.poll_lifecycle()
                self.last_state_poll = wall_time
            if wall_time - self.started > self.config.get('startup_timeout', 90):
                self.fail(f'Startup timeout; missing sensors: {missing}, active: {sorted(self.active)}')
            elif (not missing and transform_ready and footprint_ready
                  and len(self.active) == len(self.lifecycle)
                  and self.action.server_is_ready()):
                if self.ready_since is None:
                    self.ready_since = wall_time
                if wall_time - self.ready_since < 2.0:
                    return
                gps_x, gps_y = self.gps_base(pose[2])
                if math.hypot(gps_x - pose[0], gps_y - pose[1]) > 0.5:
                    self.fail('GPS and map localization disagree at startup')
                else:
                    self.send_goal()
            else:
                self.ready_since = None
            return
        if missing or not transform_ready or not footprint_ready:
            pose_age = None if pose_stamp is None else round(now - pose_stamp, 3)
            footprint_age = None if self.footprint_stamp is None else round(now - self.footprint_stamp, 3)
            self.fail(f'Stale navigation data; sensors: {missing}, TF age: {pose_age}, footprint age: {footprint_age}')
            return
        if wall_time - self.goal_started > self.config.get('goal_timeout', 120):
            self.fail('Waypoint timeout')
            return
        if wall_time - self.last_log > 5.0:
            self.get_logger().info(
                f'Waypoint {self.index + 1}/{len(self.goals)}; robot=({pose[0]:.2f}, {pose[1]:.2f})'
            )
            self.last_log = wall_time

    def send_goal(self):
        x, y, yaw = self.goals[self.index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        self.goal_started = time.monotonic()
        self.publish_status('NAVIGATING')
        self.get_logger().info(f'GPS waypoint converted to map=({x:.3f}, {y:.3f}, {yaw:.3f})')
        future = self.action.send_goal_async(NavigateToPose.Goal(pose=pose))
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.fail('Nav2 rejected the waypoint')
            elif self.state == 'FAILED':
                self.goal_handle.cancel_goal_async()
            else:
                self.goal_handle.get_result_async().add_done_callback(self.goal_result)
        except Exception as error:
            self.fail(f'Goal request failed: {error}')

    def goal_result(self, future):
        if self.state == 'FAILED':
            return
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.fail(f'Nav2 ended waypoint with status {result.status}')
                return
            x, y, yaw, _ = self.robot_pose()
            gps_x, gps_y = self.gps_base(yaw)
            goal_x, goal_y, _ = self.goals[self.index]
            if (math.hypot(x - goal_x, y - goal_y) > 0.35
                    or math.hypot(gps_x - goal_x, gps_y - goal_y) > 0.45):
                self.fail('Nav2 reported success without map/GPS arrival')
                return
            self.get_logger().info(f'Waypoint reached; map=({x:.3f}, {y:.3f}), GPS=({gps_x:.3f}, {gps_y:.3f})')
            self.goal_handle = None
            self.index += 1
            if self.index == len(self.goals):
                self.publish_status('SUCCEEDED')
            else:
                self.send_goal()
        except Exception as error:
            self.fail(f'Waypoint result failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
