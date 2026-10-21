#!/usr/bin/env python3
# Reads the waypoints from a CSV file, fits a cubic spline through them (PythonRobotics
# CubicSpline2D) and publishes the smooth path on /path with a heading on every pose.
# The raw waypoints are published on /waypoints for visualization.

import csv
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from spline.cubic_spline_planner import CubicSpline2D


class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        default_file = os.path.join(get_package_share_directory('nav'), 'waypoints', 'waypoint.csv')
        self.path_file = self.declare_parameter('path_file', default_file).value
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        # MPPI critic offsets (offset_from_furthest in mppi.yaml) are counted in path points
        # and are tuned for this spacing; change both together
        self.resolution = self.declare_parameter('path_resolution', 0.1).value

        # latched, so nodes started later (bridge, RViz) still get the path
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'path', latched)
        self.waypoints_pub = self.create_publisher(Path, 'waypoints', latched)

        waypoints = self.load_waypoints()
        if len(waypoints) < 2:
            self.get_logger().error(f'Need at least 2 waypoints in {self.path_file}')
            return

        self.waypoints_pub.publish(self.to_path([(x, y, 0.0) for x, y in waypoints]))
        self.path_pub.publish(self.to_path(self.smooth(waypoints)))

    def load_waypoints(self):
        waypoints = []
        try:
            with open(self.path_file) as f:
                for row in csv.reader(f):
                    try:
                        point = (float(row[0]), float(row[1]))
                    except (ValueError, IndexError):
                        continue  # header or empty line
                    # the spline needs distinct consecutive points
                    if not waypoints or math.dist(point, waypoints[-1]) > 1e-6:
                        waypoints.append(point)
        except OSError as e:
            self.get_logger().error(f'Failed to open path file: {e}')
        self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {self.path_file}')
        return waypoints

    def smooth(self, waypoints):
        spline = CubicSpline2D([p[0] for p in waypoints], [p[1] for p in waypoints])

        # sample the spline finely, then keep one point every `resolution` metres of real
        # distance so the spacing is even. The spline is only defined up to (not including)
        # its total length, so the last waypoint is added explicitly
        poses = []
        for s in np.arange(0.0, spline.s[-1], self.resolution / 10.0):
            x, y = spline.calc_position(s)
            if not poses or math.dist((x, y), poses[-1][:2]) >= self.resolution:
                poses.append((x, y, spline.calc_yaw(s)))
        poses.append((waypoints[-1][0], waypoints[-1][1], poses[-1][2]))

        length = sum(math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:]))
        max_curvature = max(abs(spline.calc_curvature(s)) for s in np.arange(0.0, spline.s[-1], self.resolution))
        self.get_logger().info(
            f'Smooth path: {len(poses)} points, {length:.2f} m, '
            f'tightest turn radius {1.0 / max_curvature:.2f} m')
        return poses

    def to_path(self, poses):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in poses:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        return msg


def main():
    rclpy.init()
    rclpy.spin(PathSmoother())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
