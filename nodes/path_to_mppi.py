#!/usr/bin/env python3
# Hands the smoothed path to the Nav2 controller_server (MPPI) as FollowPath action goals.
# Every CSV waypoint is a goal: /path is cut into one piece per waypoint (/waypoints) and the
# pieces are sent one after another, so the goal checker makes the robot actually reach each
# waypoint. Between waypoints MPPI may deviate (obstacles). If MPPI aborts, the rest of the
# current piece is re-sent; if a piece keeps failing for `blocked_timeout` seconds (e.g. its
# waypoint is blocked), the mission is reported FAILED and the robot stays stopped.

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath


class PathToMppi(Node):
    def __init__(self):
        super().__init__('path_to_mppi')
        self.controller_id = self.declare_parameter('controller_id', 'FollowPath').value
        self.goal_checker_id = self.declare_parameter('goal_checker_id', 'general_goal_checker').value
        self.retry_period = self.declare_parameter('retry_period', 2.0).value
        self.blocked_timeout = self.declare_parameter('blocked_timeout', 60.0).value

        self.client = ActionClient(self, FollowPath, 'follow_path')

        # latched, so the path and waypoints arrive even if the smoother started first
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, 'path', self.path_callback, latched)
        self.create_subscription(Path, 'waypoints', self.waypoints_callback, latched)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.path = None
        self.waypoints = None
        self.pieces = []          # (start_idx, end_idx) into self.path.poses, one per waypoint
        self.piece = 0            # piece currently being driven
        self.start_index = 0
        self.robot_xy = None
        self.goal_handle = None
        self.retry_timer = None
        self.failing_since = None
        self.get_logger().info('Waiting for /path and /waypoints')

    def path_callback(self, msg):
        self.path = msg
        self.start_mission()

    def waypoints_callback(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.start_mission()

    def odom_callback(self, msg):
        # odom pose is in the same "odom" frame as the path
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def start_mission(self):
        if self.path is None or self.waypoints is None or not self.path.poses:
            return
        # cut the path at the pose closest to each waypoint, searching forward only
        poses = self.path.poses
        cuts, i = [0], 0
        for wx, wy in self.waypoints[1:]:
            i = min(range(i, len(poses)),
                    key=lambda k: math.hypot(poses[k].pose.position.x - wx, poses[k].pose.position.y - wy))
            cuts.append(i)
        self.pieces = [(a, b) for a, b in zip(cuts, cuts[1:]) if b > a]
        self.piece, self.start_index, self.failing_since = 0, 0, None
        self.get_logger().info(f'Path with {len(poses)} points split into {len(self.pieces)} pieces at waypoints')
        self.send_goal()

    def remaining_piece(self):
        # MPPI only searches the start of a new path for the robot, so drop the part of the
        # current piece already driven: start from the closest point, searching forward only
        start, end = self.pieces[self.piece]
        self.start_index = max(self.start_index, start)
        if self.robot_xy is not None:
            rx, ry = self.robot_xy
            poses = self.path.poses
            self.start_index = min(
                range(self.start_index, end + 1),
                key=lambda k: math.hypot(poses[k].pose.position.x - rx, poses[k].pose.position.y - ry))
        piece = Path()
        piece.header = self.path.header
        piece.poses = self.path.poses[self.start_index:end + 1]
        if len(piece.poses) < 2:  # already at the end of this piece
            piece.poses = self.path.poses[end - 1:end + 1]
        return piece

    def send_goal(self):
        if not self.client.server_is_ready():
            self.get_logger().warn(f'controller_server not ready, retrying in {self.retry_period:.1f}s')
            self.schedule_retry()
            return

        path = self.remaining_piece()
        wx, wy = self.waypoints[self.piece + 1]
        self.get_logger().info(f'Piece {self.piece + 1}/{len(self.pieces)} -> waypoint ({wx:.2f}, {wy:.2f}), '
                               f'{len(path.poses)} points')

        # smoother may run on wall time while the controller runs on sim time
        stamp = self.get_clock().now().to_msg()
        path.header.stamp = stamp
        for pose in path.poses:
            pose.header.stamp = stamp

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        self.client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Path rejected by controller_server')
            self.schedule_retry()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.result_callback(handle, f.result().status))

    def result_callback(self, handle, status):
        # ignore results of goals that were replaced by a newer one
        if handle is not self.goal_handle:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            wx, wy = self.waypoints[self.piece + 1]
            error = math.hypot(self.robot_xy[0] - wx, self.robot_xy[1] - wy) if self.robot_xy else float('nan')
            self.get_logger().info(f'Reached waypoint {self.piece + 1} ({wx:.2f}, {wy:.2f}), error {error:.3f} m')
            self.failing_since = None
            self.piece += 1
            if self.piece == len(self.pieces):
                self.get_logger().info('Successfully reached goal (all waypoints visited)')
                return
            self.send_goal()
        elif status == GoalStatus.STATUS_ABORTED:
            now = self.get_clock().now().nanoseconds / 1e9
            self.failing_since = self.failing_since or now
            if now - self.failing_since > self.blocked_timeout:
                wx, wy = self.waypoints[self.piece + 1]
                self.get_logger().error(f'MISSION FAILED: waypoint ({wx:.2f}, {wy:.2f}) unreachable '
                                        f'for {self.blocked_timeout:.0f}s (blocked?). Robot stopped.')
                return
            self.get_logger().warn(f'MPPI aborted (blocked?), re-sending in {self.retry_period:.1f}s')
            self.schedule_retry()
        else:
            self.get_logger().warn('Path goal canceled')

    def schedule_retry(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
        self.retry_timer = self.create_timer(self.retry_period, self.retry)

    def retry(self):
        self.retry_timer.cancel()
        self.send_goal()


def main():
    rclpy.init()
    rclpy.spin(PathToMppi())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
