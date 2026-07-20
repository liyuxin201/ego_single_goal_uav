#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def default_px4_root():
    return Path(__file__).resolve().parents[1] / ".." / "PX4-Autopilot"


def parse_args():
    parser = argparse.ArgumentParser(description="Start the local PX4 SITL binary.")
    parser.add_argument("--px4-root", default=str(default_px4_root()))
    parser.add_argument("--build-dir", default="build/px4_sitl_default")
    parser.add_argument("--sim-model", default="gazebo-classic_iris")
    parser.add_argument("--instance", default="0")
    args, _ros_remaps = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    px4_root = Path(args.px4_root).expanduser().resolve()
    build_dir = px4_root / args.build_dir
    px4_bin = build_dir / "bin" / "px4"
    rootfs = build_dir / "etc"

    if not px4_bin.is_file():
        print("PX4 SITL binary not found: {}".format(px4_bin), file=sys.stderr)
        print("Build it with: cd {} && make px4_sitl_default".format(px4_root), file=sys.stderr)
        return 1

    if not rootfs.is_dir():
        print("PX4 SITL rootfs not found: {}".format(rootfs), file=sys.stderr)
        return 1

    run_dir = build_dir / "tmp" / "sitl_{}".format(args.instance)
    run_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["PX4_SIM_MODEL"] = args.sim_model
    env["PATH"] = "{}:{}".format(build_dir / "bin", env.get("PATH", ""))

    command = [
        str(px4_bin),
        str(rootfs),
        "-s",
        "etc/init.d-posix/rcS",
        "-i",
        str(args.instance),
        "-w",
        str(run_dir),
    ]

    child = subprocess.Popen(command, cwd=str(run_dir), env=env, stdin=subprocess.PIPE)

    def forward_signal(signum, _frame):
        if child.poll() is None:
            child.send_signal(signum)

    signal.signal(signal.SIGINT, forward_signal)
    signal.signal(signal.SIGTERM, forward_signal)

    started_at = time.monotonic()
    rc = child.wait()
    runtime = time.monotonic() - started_at

    # Some prebuilt PX4 SITL runtimes report rc=2 after the simulator bridge
    # has started and is waiting for Gazebo. Treat that as non-fatal so
    # roslaunch does not tear down the whole simulation during startup.
    if rc == 2 and runtime > 1.0:
        print(
            "PX4 SITL exited with code 2 after startup; treating as non-fatal",
            file=sys.stderr,
        )
        return 0

    return rc


if __name__ == "__main__":
    sys.exit(main())
