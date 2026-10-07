import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, NavSatFix, PointCloud2


def main(args=None):
    rclpy.init(args=args)
    node = Node('simulation_ready')
    received = {}
    subscriptions = []

    def observe(key, message):
        received[key] = message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    for topic, message_type in (
        ('/odom', Odometry), ('/scan', LaserScan), ('/gps/fix', NavSatFix),
        ('/camera/depth_camera/points', PointCloud2),
    ):
        subscriptions.append(node.create_subscription(
            message_type, topic,
            lambda message, topic=topic: observe(topic, message),
            qos_profile_sensor_data,
        ))
    started = time.monotonic()
    ready_since = None
    success = False
    try:
        while rclpy.ok() and time.monotonic() - started < 60:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = node.get_clock().now().nanoseconds / 1e9
            ready = now > 0 and len(received) == 4 and all(
                -0.5 <= now - stamp <= 1.0 for stamp in received.values()
            )
            if ready:
                if ready_since is None:
                    ready_since = time.monotonic()
                if time.monotonic() - ready_since >= 1.0:
                    success = True
                    node.get_logger().info('Simulation clock and all four sensor streams are ready')
                    break
            else:
                ready_since = None
        if not success:
            node.get_logger().error(f'Simulation readiness timeout; received: {sorted(received)}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)
