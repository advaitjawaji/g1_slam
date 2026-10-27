#!/usr/bin/env python3
"""
Send one Nav2 goal and log it into the current trial.

The goal pose is published on /g1/trial/goal, and goal_sent / goal_accepted /
goal_result events on /g1/trial/events (both recorded in the trial rosbag and
mirrored to <trial>/events.jsonl). That gives every trial its desired end
pose and outcome, which a goal clicked in RViz does not reliably leave behind.

    python3 tools/send_goal.py 3.0 0.0            # x y in the odom frame
    python3 tools/send_goal.py 3.0 1.0 1.57       # with final yaw (rad)
    python3 tools/send_goal.py 3.0 0.0 --timeout 60

Ctrl-C cancels the goal and logs goal_canceled_by_operator.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from trial_event import LATCHED, EventLog

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def _init_rclpy():
    # Handle Ctrl-C ourselves so the goal can still be cancelled over ROS.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("yaw", type=float, nargs="?", default=0.0, help="final heading, rad")
    ap.add_argument("--frame", default="odom", help="goal frame (default odom)")
    ap.add_argument("--timeout", type=float, default=0.0,
                    help="cancel after this many seconds (0 = wait indefinitely)")
    args = ap.parse_args()

    _init_rclpy()
    node = Node("trial_goal_sender")
    log = EventLog(node)
    goal_pub = node.create_publisher(PoseStamped, "/g1/trial/goal", LATCHED)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    rc = 1
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            log.emit("goal_error", reason="navigate_to_pose action server not available")
            return 1

        pose = PoseStamped()
        pose.header.frame_id = args.frame
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = args.x
        pose.pose.position.y = args.y
        pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        goal_pub.publish(pose)
        log.emit("goal_sent", x=args.x, y=args.y, yaw=args.yaw, frame=args.frame)
        send_fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_fut)
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            log.emit("goal_rejected")
            return 1
        log.emit("goal_accepted")

        res_fut = handle.get_result_async()
        t0 = time.time()
        try:
            while not res_fut.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                if args.timeout and time.time() - t0 > args.timeout:
                    log.emit("goal_timeout", after_s=args.timeout)
                    rclpy.spin_until_future_complete(node, handle.cancel_goal_async(),
                                                     timeout_sec=3.0)
                    break
        except KeyboardInterrupt:
            log.emit("goal_canceled_by_operator", after_s=round(time.time() - t0, 2))
            rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=3.0)

        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=3.0)
        if res_fut.done() and res_fut.result() is not None:
            status = res_fut.result().status
            log.emit("goal_result", status=STATUS_NAMES.get(status, str(status)),
                     duration_s=round(time.time() - t0, 2))
            rc = 0 if status == GoalStatus.STATUS_SUCCEEDED else 1
        return rc
    finally:
        log.linger()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
