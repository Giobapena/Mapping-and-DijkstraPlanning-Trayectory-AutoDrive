#!/usr/bin/env python3
"""Publica la trayectoria cruda y la suavizada como nav_msgs/Path + marcadores."""
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def read_csv(path):
    pts = []
    with open(path, 'r') as f:
        for row in csv.DictReader(f):
            yaw = float(row['yaw']) if row.get('yaw') else None
            pts.append([float(row['x']), float(row['y']), yaw])
    for i in range(len(pts)):
        if pts[i][2] is None:
            j = min(i + 1, len(pts) - 1)
            pts[i][2] = math.atan2(pts[j][1] - pts[i][1], pts[j][0] - pts[i][0])
    return pts


class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')
        self.declare_parameter('smooth_csv', '')
        self.declare_parameter('raw_csv', '')
        self.declare_parameter('frame_id', 'map')
        self.frame = self.get_parameter('frame_id').value

        # Transient local: RViz recibe la ruta aunque se conecte despues
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_s = self.create_publisher(Path, '/global_path', qos)
        self.pub_r = self.create_publisher(Path, '/global_path_raw', qos)
        self.pub_m = self.create_publisher(MarkerArray, '/global_path_markers', qos)

        s_csv = self.get_parameter('smooth_csv').value
        self.smooth = self._path(s_csv)
        self.raw = self._path(self.get_parameter('raw_csv').value)
        self.markers = self._markers(s_csv)

        self.create_timer(1.0, self.tick)
        n = len(self.smooth.poses) if self.smooth else 0
        self.get_logger().info('Publicando /global_path (%d poses) en frame "%s"'
                               % (n, self.frame))

    def _path(self, csv_path):
        if not csv_path or not os.path.exists(csv_path):
            self.get_logger().warn('CSV no encontrado: %r' % csv_path)
            return None
        msg = Path()
        msg.header.frame_id = self.frame
        for x, y, yaw in read_csv(csv_path):
            p = PoseStamped()
            p.header.frame_id = self.frame
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(p)
        return msg

    def _markers(self, csv_path):
        if not csv_path or not os.path.exists(csv_path):
            return None
        pts = read_csv(csv_path)
        arr = MarkerArray()

        line = Marker()
        line.header.frame_id = self.frame
        line.ns, line.id, line.type = 'trajectory', 0, Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color = ColorRGBA(r=0.15, g=0.85, b=0.30, a=1.0)
        line.pose.orientation.w = 1.0
        line.points = [Point(x=p[0], y=p[1], z=0.05) for p in pts]
        arr.markers.append(line)

        for i, (idx, rgb) in enumerate([(0, (0.1, 1.0, 0.1)), (-1, (1.0, 0.1, 0.1))]):
            m = Marker()
            m.header.frame_id = self.frame
            m.ns, m.id, m.type = 'endpoints', i + 1, Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pts[idx][0]
            m.pose.position.y = pts[idx][1]
            m.pose.position.z = 0.10
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.28
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
            arr.markers.append(m)
        return arr

    def tick(self):
        now = self.get_clock().now().to_msg()
        for msg, pub in ((self.smooth, self.pub_s), (self.raw, self.pub_r)):
            if msg is None:
                continue
            msg.header.stamp = now
            for p in msg.poses:
                p.header.stamp = now
            pub.publish(msg)
        if self.markers is not None:
            for m in self.markers.markers:
                m.header.stamp = now
            self.pub_m.publish(self.markers)


def main():
    rclpy.init()
    node = GlobalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
