#!/usr/bin/env python3
"""
Mark a moment in the current trial.

The event is published on /g1/trial/events (so it lands in the trial rosbag on
the same clock as everything else) and appended to <trial>/events.jsonl.

    python3 tools/trial_event.py "pedestrian 2 starts walking"
    python3 tools/trial_event.py --kind failure "robot did not stop for P1"
    python3 tools/trial_event.py --kind estop "operator pressed e-stop"

The current trial is the one run_zed_e2e.sh is recording
(~/.g1_zed_current_trial). Also imported by tools/send_goal.py.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

CURRENT_TRIAL_FILE = os.path.expanduser("~/.g1_zed_current_trial")
EVENTS_TOPIC = "/g1/trial/events"

# Transient-local so the bag recorder still receives the message if it matches
# this short-lived publisher a moment after it published.
LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def current_trial_dir() -> str | None:
    try:
        with open(CURRENT_TRIAL_FILE) as f:
            d = f.read().strip()
    except OSError:
        return None
    return d if os.path.isdir(d) else None


class EventLog:
    def __init__(self, node: Node, wait_for_recorder_s: float = 3.0):
        self._node = node
        self._pub = node.create_publisher(String, EVENTS_TOPIC, LATCHED)
        self.trial_dir = current_trial_dir()
        deadline = time.time() + wait_for_recorder_s
        while self._pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if self._pub.get_subscription_count() == 0:
            print("[trial_event] no subscriber on /g1/trial/events — is the trial recorder "
                  "running? Writing events.jsonl only.", flush=True)

    def emit(self, kind: str, **fields) -> dict:
        rec = {"t": self._node.get_clock().now().nanoseconds, "kind": kind, **fields}
        msg = String()
        msg.data = json.dumps(rec, separators=(",", ":"))
        self._pub.publish(msg)
        if self.trial_dir:
            with open(os.path.join(self.trial_dir, "events.jsonl"), "a") as f:
                f.write(msg.data + "\n")
        print(f"[trial_event] {kind}: {fields}", flush=True)
        return rec

    def linger(self, seconds: float = 0.5):
        """Keep the publisher alive briefly so the recorder can take the message."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self._node, timeout_sec=0.05)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("text", help="what happened")
    ap.add_argument("--kind", default="note",
                    help="event type, e.g. note / failure / estop / ped_start (default note)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("trial_event")
    log = EventLog(node)
    if log.trial_dir is None:
        print("[trial_event] no active trial folder (~/.g1_zed_current_trial); "
              "event published to ROS only.", flush=True)
    log.emit(args.kind, text=args.text)
    log.linger()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
