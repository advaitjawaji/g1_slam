#!/usr/bin/env python3
"""
Snapshot the running ROS graph into <trial>/snapshot, from one process:

    nodes.txt              every node (as `ros2 node list`)
    topics.txt             every topic with its type (as `ros2 topic list -t`)
    params/<node>.yaml     each node's live parameters (as `ros2 param dump`)
    lifecycle.txt          state of every lifecycle node (Nav2: should be active)
    tf/frames.yaml, .gv    the TF tree: parent, broadcaster and rate per frame
    tf/odom_to_pelvis.txt  odom -> pelvis right now (as `tf2_echo`)

This replaces ~30 `ros2 param dump` CLI calls run one after another (15 s
timeout each). On the Jetson Thor they never finished before a trial ended,
so no trial on 2026-10-27 got a TF tree or the ZED's parameters, and they kept
spawning CLI processes while the robot navigated. This takes a few seconds.

    python3 tools/graph_snapshot.py <trial_dir>
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import yaml

# ParameterType constants (rcl_interfaces/msg/ParameterType); kept here so the
# pure helpers below are testable without ROS.
_VALUE_FIELDS = {
    1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value",
    5: "byte_array_value", 6: "bool_array_value", 7: "integer_array_value",
    8: "double_array_value", 9: "string_array_value",
}


def param_value(v):
    """rcl_interfaces/ParameterValue -> plain Python value (None if NOT_SET)."""
    field = _VALUE_FIELDS.get(v.type)
    if field is None:
        return None
    val = getattr(v, field)
    return list(val) if v.type >= 5 else val


def nest(flat: dict) -> dict:
    """{'a.b.c': 1} -> {'a': {'b': {'c': 1}}}, as `ros2 param dump` writes it."""
    out: dict = {}
    for name in sorted(flat):
        parts = name.split(".")
        d = out
        for p in parts[:-1]:
            if not isinstance(d.get(p), dict):
                d[p] = {}
            d = d[p]
        d[parts[-1]] = flat[name]
    return out


def frames_gv(frames: dict) -> str:
    """TF tree (tf2 Buffer.all_frames_as_yaml) -> graphviz, edges parent -> child."""
    lines = ["digraph tf {"]
    for child in sorted(frames):
        info = frames[child] or {}
        label = f"{info.get('broadcaster', '?')}\\n{info.get('rate', '?')} Hz"
        lines.append(f'  "{info.get("parent", "?")}" -> "{child}" [label="{label}"];')
    lines.append("}")
    return "\n".join(lines) + "\n"


def file_name(node_fqn: str) -> str:
    return node_fqn.lstrip("/").replace("/", "_") + ".yaml"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trial_dir")
    ap.add_argument("--tf-wait", type=float, default=3.0, help="seconds to listen to /tf")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rcl_interfaces.srv import GetParameters, ListParameters
    from lifecycle_msgs.srv import GetState
    import tf2_ros

    d = os.path.join(args.trial_dir, "snapshot")
    os.makedirs(os.path.join(d, "params"), exist_ok=True)
    os.makedirs(os.path.join(d, "tf"), exist_ok=True)

    rclpy.init()
    node = Node("trial_graph_snapshot")
    t_start = time.monotonic()
    # The TF listener's subscriptions are served by every spin_once below.
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_for(1.0)                                     # let graph discovery settle

    def call(srv_type, name, request, timeout=2.0):
        cli = node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=0.5):
                return None
            fut = cli.call_async(request)
            end = time.monotonic() + timeout
            while not fut.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            return fut.result() if fut.done() else None
        finally:
            node.destroy_client(cli)

    me = node.get_fully_qualified_name()
    nodes = sorted({(ns.rstrip("/") + "/" + n) for n, ns in node.get_node_names_and_namespaces()} - {me})
    with open(os.path.join(d, "nodes.txt"), "w") as f:
        f.write("\n".join(nodes) + "\n")
    with open(os.path.join(d, "topics.txt"), "w") as f:
        for t, types in sorted(node.get_topic_names_and_types()):
            f.write(f"{t} [{', '.join(types)}]\n")

    services = {name for name, _ in node.get_service_names_and_types()}
    failed, lifecycle = [], []
    for fqn in nodes:
        if f"{fqn}/list_parameters" in services:
            req = ListParameters.Request(); req.depth = 0          # 0 = recursive
            lst = call(ListParameters, f"{fqn}/list_parameters", req)
            names = list(lst.result.names) if lst else None
            got = None
            if names is not None:
                req = GetParameters.Request(); req.names = names
                got = call(GetParameters, f"{fqn}/get_parameters", req, timeout=4.0)
            if got is None:
                failed.append(fqn)
            else:
                flat = {n: param_value(v) for n, v in zip(names, got.values)}
                with open(os.path.join(d, "params", file_name(fqn)), "w") as f:
                    yaml.safe_dump({fqn: {"ros__parameters": nest(flat)}}, f,
                                   default_flow_style=False, sort_keys=True)
        if f"{fqn}/get_state" in services:
            st = call(GetState, f"{fqn}/get_state", GetState.Request())
            lifecycle.append(f"{fqn}: {st.current_state.label if st else 'no reply'}")
    with open(os.path.join(d, "lifecycle.txt"), "w") as f:
        f.write("\n".join(lifecycle) + ("\n" if lifecycle else ""))

    # TF: the listener has been collecting since the start.
    spin_for(max(0.0, args.tf_wait - (time.monotonic() - t_start)))
    frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()) or {}
    with open(os.path.join(d, "tf", "frames.yaml"), "w") as f:
        yaml.safe_dump(frames, f, default_flow_style=False, sort_keys=True)
    with open(os.path.join(d, "tf", "frames.gv"), "w") as f:
        f.write(frames_gv(frames))
    with open(os.path.join(d, "tf", "odom_to_pelvis.txt"), "w") as f:
        try:
            tr = tf_buffer.lookup_transform("odom", "pelvis", Time()).transform
            t, q = tr.translation, tr.rotation
            f.write(f"Translation: [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]\n"
                    f"Rotation: in Quaternion [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]\n")
        except Exception as exc:                       # TF errors vary by distro
            f.write(f"lookup failed: {exc}\n")

    not_active = [ln for ln in lifecycle if not ln.endswith(": active")]
    print(f"[snapshot] {len(nodes)} nodes, {len(nodes) - len(failed)} parameter dumps, "
          f"{len(frames)} TF frames, {len(lifecycle)} lifecycle nodes "
          f"({len(not_active)} not active) in {time.monotonic() - t_start:.1f} s", flush=True)
    for ln in not_active:
        print(f"[snapshot] NOT ACTIVE  {ln}", flush=True)
    if failed:
        print(f"[snapshot] no parameters from: {', '.join(failed)}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
