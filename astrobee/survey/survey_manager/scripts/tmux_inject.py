#!/usr/bin/env python3

# Copyright (c) 2023, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
A wrapper that injects the specified child command into a tmux target window, waits for the child to
exit, and returns its exit code. This enables us to run a command with a terminal interface
automatically from within a ROS service while retaining the option for an operator to use that
terminal interface later by attaching to the relevant tmux session.

Example usage:
In terminal 1: tmux_inject.py -s 1 -w 5 -- ipython3 --no-banner  # Inject command in tmux window
In terminal 2: tmux attach -s 1  # Then change to window 5 to interact with ipython3; exit ipython3
In terminal 1: echo $?  # tmux_inject.py exits after ipython3 exits. Echo return code of ipython3.

Note that we used the '--' separator to ensure that tmux_inject.py doesn't try to capture/interpret
flags after that point that are intended for the child command.

Behavior details:

- Caveat: If targeting multiple tmux_inject.py commands at the same tmux window, you must ensure
  each command completes before running the next command. (When two commands try to take over the
  same terminal, you just get a mess.)

- The targeted tmux session (-s) and window (-w) are created if they don't already exist.

- It's a bit tricky to propagate the return code from the child command because its parent process
  is the shell running in the tmux window, not tmux_inject.py. To make this work, when we inject the
  specified command into the tmux window, we wrap it in the pidwrap.sh script, which exposes its pid
  and return code.

- The tmux_inject.py script will try to kill the child if it is forced to exit while the child is
  still running. (This doesn't happen automatically because the child's parent process is the tmux
  window shell.)
"""

import argparse
import logging
import os
import pathlib
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import List

THIS_DIR = pathlib.Path(__file__).resolve().parent
WINDOW_EXISTED_REGEX = re.compile(r"^create window failed: index \d+ in use\n$")


class TmuxTimeoutError(RuntimeError):
    """
    Represents a timeout exception raised from tmux_inject.py.
    """


def run(
    args: List[str], capture_output: bool = False, check: bool = False
) -> subprocess.CompletedProcess:
    """
    Wrapper for subprocess.run() that supplies default encoding and logs each command.
    """
    logging.info("+ %s", shlex.join(args))
    return subprocess.run(
        args, capture_output=capture_output, encoding="utf-8", check=check
    )


def wait_until_path_exists(
    path: pathlib.Path, timeout_seconds: float, check_period_seconds: float = 0.1
) -> None:
    """
    Wait until `path` exists for up to `timeout_seconds`. Poll for existence every
    'check_period_seconds`. Raise TmuxTimeoutError on timeout.
    """
    start_time = time.time()
    while True:
        if path.exists():
            return
        elapsed_time = time.time() - start_time
        if elapsed_time > timeout_seconds:
            raise TmuxTimeoutError(
                f"wait_until_path_exists(): File {path} does not exist after timeout of {timeout_seconds} seconds"
            )
        time.sleep(check_period_seconds)


def tmux_inject(session: str, window: str, command_args: List[str]) -> None:
    """
    The main driver function.
    """
    proc = run(
        ["tmux", "new-session", "-d", "-s", session], capture_output=True, check=False
    )
    session_existed = "duplicate session:" in proc.stderr
    if session_existed:
        logging.info("[Target session already existed, continuing]")
    if proc.returncode != 0 and not session_existed:
        logging.warning("WARNING: tmux new-session: %s", proc.stderr)

    proc = run(
        ["tmux", "new-window", "-d", "-t", f"{session}:{window}"],
        capture_output=True,
        check=False,
    )
    window_existed = bool(WINDOW_EXISTED_REGEX.search(proc.stderr))
    if window_existed:
        logging.info("[Target window already existed, continuing]")
    if proc.returncode != 0 and not window_existed:
        logging.warning("WARNING: tmux new-window: %s", proc.stderr)

    # Create unique temp dir and copy pidwrap.sh into it
    pidwrap_src_path = THIS_DIR / "pidwrap.sh"
    temp_dir = pathlib.Path(tempfile.mkdtemp(prefix="tmux_inject_"))
    pidwrap_dst_path = temp_dir / "pidwrap.sh"
    shutil.copy(pidwrap_src_path, pidwrap_dst_path)

    shell_cmd = shlex.join([str(pidwrap_dst_path)] + command_args)
    logging.info("Starting child command in tmux window.")
    run(
        ["tmux", "send-keys", "-t", f"{session}:{window}", shell_cmd, "Enter"],
        check=True,
    )

    pid_path = temp_dir / "pid.txt"
    wait_until_path_exists(pid_path, timeout_seconds=1.0)
    pid = int(pid_path.read_text(encoding="utf-8"))
    try:
        logging.info("Child has PID %s. Waiting for child to exit.", pid)
        run(["tail", "-f", f"--pid={pid}", "/dev/null"], check=True)
    finally:
        # This branch runs every time but really exists for the case that the run() command above
        # exits before the child process finished. (Like if the tail command or this script received
        # a signal, perhaps user typed Ctrl-C.)
        pid_status_path = pathlib.Path(f"/proc/{pid}")
        if pid_status_path.is_dir():
            logging.warning(
                "%s",
                "\nEarly exit while child may still be running. Trying to kill child.",
            )
            logging.info("+ kill -TERM %s", pid)
            os.kill(pid, signal.SIGTERM)

    return_code_path = temp_dir / "return_code.txt"
    wait_until_path_exists(return_code_path, timeout_seconds=1.0)
    return_code = int(return_code_path.read_text(encoding="utf-8"))
    logging.info("Child exited with return code %s. Exiting.", return_code)
    sys.exit(return_code)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    """
    Custom formatter for argparse that combines formatting mixins.
    """


def main():
    """
    Parse command line arguments and call tmux_inject() main driver.
    """
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-s",
        "--session",
        help="Tmux session name to target. See target-session in 'man tmux'.",
        default="1",
    )
    parser.add_argument(
        "-w",
        "--window",
        help="Tmux window to target. See target-window in 'man tmux'.",
        default="5",
    )
    parser.add_argument(
        "arg", nargs="+", help="Arguments of the command to inject into the tmux window"
    )
    args = parser.parse_args()

    logging.basicConfig(format="%(message)s", level=logging.INFO)
    tmux_inject(session=args.session, window=args.window, command_args=args.arg)


if __name__ == "__main__":
    main()
