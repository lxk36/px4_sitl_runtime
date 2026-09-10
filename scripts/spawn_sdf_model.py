#!/usr/bin/env python3
"""Spawn one SDF model into Gazebo, owning preflight, replace, and verify.

The orchestrator previously forked three Python processes per robot
(rosservice preflight, this spawner, rosservice verification). One rospy
process now performs the whole sequence, so each robot costs one interpreter
start instead of three.

gzserver dispatches every /gazebo/spawn_* call on one callback thread.
Overlapping inserts often return success and then drop the model, so this
helper holds an exclusive flock across delete/spawn/verify, retries a silent
drop, and shares the lock file with Mecanum's gazebo_ros spawn_model prefix.

Exit codes:
  0  the model was spawned and is visible in Gazebo
  4  the model already exists and --existing-model-policy is "fail"
     (permanent: retrying without operator action cannot succeed)
  1  any other failure (transient: Gazebo or ROS may still be starting)
"""
import argparse
import fcntl
import json
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

import rospy
from gazebo_msgs.srv import DeleteModel, GetModelState, SpawnModel
from geometry_msgs.msg import Pose

EXIT_TRANSIENT = 1
EXIT_MODEL_EXISTS = 4
SERVICE_WAIT_SECONDS = 30.0
DELETE_WAIT_SECONDS = 5.0
VERIFY_WAIT_SECONDS = 10.0
SPAWN_ATTEMPTS = 4
SPAWN_RETRY_SECONDS = 0.5
SETTLE_SECONDS = 0.4
GAZEBO_SPAWN_LOCK_PATH = "/tmp/xgc2-gazebo-spawn.lock"


def request_cancel(_signum, _frame):
    raise SystemExit(EXIT_TRANSIENT)


def _set_plugin_tag(plugin, tag, value):
    elem = plugin.find(tag)
    if elem is not None:
        elem.text = str(value)


def render_sdf(path, mavlink_tcp_port, mavlink_udp_port, qgc_udp_port, sdk_udp_port):
    tree = ET.parse(path)
    root = tree.getroot()
    for plugin in root.iter("plugin"):
        if plugin.attrib.get("name") == "mavlink_interface":
            _set_plugin_tag(plugin, "mavlink_tcp_port", mavlink_tcp_port)
            _set_plugin_tag(plugin, "mavlink_udp_port", mavlink_udp_port)
            _set_plugin_tag(plugin, "qgc_udp_port", qgc_udp_port)
            _set_plugin_tag(plugin, "sdk_udp_port", sdk_udp_port)
    return ET.tostring(root, encoding="unicode")


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def service_proxy(name, service_type):
    rospy.wait_for_service(name, timeout=SERVICE_WAIT_SECONDS)
    return rospy.ServiceProxy(name, service_type)


def acquire_gazebo_spawn_lock():
    """Serialize /gazebo/spawn_* against one gzserver.

    Job workers and SITL instances stay parallel. The lock file is the same
    path Mecanum launch-prefix flock(1) uses, so UAV and UGV inserts queue.
    """
    handle = open(GAZEBO_SPAWN_LOCK_PATH, "a+")
    fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
    return handle


def model_exists(get_model_state, model):
    return bool(get_model_state(model, "world").success)


def wait_for_model(get_model_state, model, present, deadline_seconds):
    deadline = time.monotonic() + deadline_seconds
    while True:
        if model_exists(get_model_state, model) == present:
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.1)


def spawn_until_visible(spawn, get_model_state, args, sdf, pose):
    last_message = ""
    for attempt in range(1, SPAWN_ATTEMPTS + 1):
        if model_exists(get_model_state, args.model):
            return
        result = spawn(args.model, sdf, "", pose, "world")
        last_message = result.status_message
        if result.success and wait_for_model(
            get_model_state, args.model, True, VERIFY_WAIT_SECONDS
        ):
            time.sleep(SETTLE_SECONDS)
            rospy.loginfo("SpawnModel: %s", result.status_message)
            return
        if not result.success:
            if args.existing_model_policy == "fail" and model_exists(
                get_model_state, args.model
            ):
                rospy.logerr("model %s already exists", args.model)
                raise SystemExit(EXIT_MODEL_EXISTS)
            rospy.logwarn(
                "SpawnModel attempt %s failed: %s", attempt, result.status_message
            )
        else:
            rospy.logwarn(
                "Gazebo did not report model %s after spawn attempt %s",
                args.model,
                attempt,
            )
        if attempt < SPAWN_ATTEMPTS:
            time.sleep(SPAWN_RETRY_SECONDS)
    rospy.logerr(
        "Gazebo did not keep model %s after %s spawn attempts (%s)",
        args.model,
        SPAWN_ATTEMPTS,
        last_message,
    )
    raise SystemExit(EXIT_TRANSIENT)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--mavlink-tcp-port", type=int, required=True)
    parser.add_argument("--mavlink-udp-port", type=int, required=True)
    parser.add_argument("--qgc-udp-port", type=int, required=True)
    parser.add_argument("--sdk-udp-port", type=int, required=True)
    parser.add_argument(
        "--existing-model-policy", choices=("fail", "replace"), default="fail"
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    signal.signal(signal.SIGTERM, request_cancel)
    signal.signal(signal.SIGINT, request_cancel)
    rospy.init_node("spawn_sdf_model", anonymous=True, disable_signals=True)
    sdf = render_sdf(
        args.sdf,
        args.mavlink_tcp_port,
        args.mavlink_udp_port,
        args.qgc_udp_port,
        args.sdk_udp_port,
    )

    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    qx, qy, qz, qw = quaternion_from_rpy(args.roll, args.pitch, args.yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    lock = acquire_gazebo_spawn_lock()
    try:
        get_model_state = service_proxy("/gazebo/get_model_state", GetModelState)
        replaced = False
        if args.existing_model_policy == "replace" and model_exists(
            get_model_state, args.model
        ):
            delete_model = service_proxy("/gazebo/delete_model", DeleteModel)
            result = delete_model(args.model)
            if not result.success:
                rospy.logerr("DeleteModel failed: %s", result.status_message)
                raise SystemExit(EXIT_TRANSIENT)
            if not wait_for_model(get_model_state, args.model, False, DELETE_WAIT_SECONDS):
                rospy.logerr("model %s still exists after delete_model", args.model)
                raise SystemExit(EXIT_TRANSIENT)
            replaced = True

        spawn = service_proxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawn_until_visible(spawn, get_model_state, args, sdf, pose)
        print("SPAWN_SDF_RESULT " + json.dumps({"replaced": replaced}), flush=True)
    finally:
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        lock.close()


if __name__ == "__main__":
    main()
