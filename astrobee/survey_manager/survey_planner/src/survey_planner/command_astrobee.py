#!/usr/bin/env python3
#
# Copyright (c) 2021, United States Government, as represented by the
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

import argparse
import fcntl
import os
import pathlib
import socket
import subprocess
import sys
import threading
import time  # Add time module for waiting
from dataclasses import dataclass
from typing import List

import rospkg
import rospy
from ff_msgs.msg import (
    AckCompletedStatus,
    AckStamped,
    CommandArg,
    CommandConstants,
    CommandStamped,
    PlanStatusStamped,
)
from std_msgs.msg import Header, String

# Imports from survey_planner package
from survey_planner.problem_generator import (
    YamlMapping,
    load_yaml,
    yaml_action_from_pddl,
)

# Constants
MAX_COUNTER = 10
CHUNK_SIZE = 1024

# Note: INFO_CONTEXT may get extended below during arg parsing
INFO_CONTEXT = "command_astrobee"


def loginfo(msg: str) -> None:
    "Call rospy.loginfo() with context."
    rospy.loginfo(f"{INFO_CONTEXT}: {msg}")


def first_non_zero(a: int, b: int) -> int:
    "Return the first non-zero between `a` and `b`, or zero if both are zero."
    if a != 0:
        return a
    return b


def exposure_change(config_static, bay_origin, bay_destination):
    # Going to JEM
    if bay_origin == "nod2_hatch_to_jem" and bay_destination == "jem_hatch_from_nod2":
        loginfo("CHANGING EXPOSURE TO JEM")
        return config_static["exposure"]["jem"]

    # Going  to NOD2
    if (
        bay_origin == "jem_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_jem"
        or bay_origin == "usl_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_usl"
    ):
        loginfo("CHANGING EXPOSURE TO NOD2")
        return config_static["exposure"]["nod2"]

    # Going to USL
    if bay_origin == "nod2_hatch_to_usl" and bay_destination == "usl_hatch_from_nod2":
        return config_static["exposure"]["usl"]

    return 0


def map_change(config_static, bay_origin, bay_destination):
    # Going to JEM
    if bay_origin == "nod2_hatch_to_jem" and bay_destination == "jem_hatch_from_nod2":
        loginfo("CHANGING MAP TO JEM")
        return config_static["maps"]["jem"]
    # Going  to NOD2
    if (
        bay_origin == "jem_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_jem"
        or bay_origin == "usl_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_usl"
    ):
        loginfo("CHANGING MAP TO NOD2")
        return config_static["maps"]["nod2"]
    # Going to USL
    if bay_origin == "nod2_hatch_to_usl" and bay_destination == "usl_hatch_from_nod2":
        return config_static["maps"]["usl"]
    return ""


def get_ops_plan_path():
    # Check if the path /opt/astrobee/ops/gds/plans/ exists
    if os.path.exists("/opt/astrobee/ops/gds/plans/"):
        return "/opt/astrobee/ops/gds/plans/"

    # Check if the environment variable $ASTROBEE_OPS exists
    astrobee_ops_path = os.getenv("ASTROBEE_OPS")
    if astrobee_ops_path:
        return os.path.join(astrobee_ops_path, "dock_scripts/hsc/gds/plans")

    # Check if the symlink ~/gds/latest/ControlStationConfig/IssWorld exists
    symlink_path = os.path.expanduser("~/gds/latest/ControlStationConfig/IssWorld")
    if os.path.islink(symlink_path):
        # Get the target of the symlink
        target_path = os.path.realpath(symlink_path)
        # Construct the relative path ../../plans
        relative_path = os.path.join(target_path, "../../dock_scripts/hsc/gds/plans")
        return relative_path

    # Return None if none of the conditions are met
    return None


def get_ops_path() -> pathlib.Path:
    return pathlib.Path(get_ops_plan_path()).parent.parent.resolve()


# This class starts a new process and lets you monitor the input and output
# Mostly used for actions where user inteference might be required
class ProcessExecutor:
    def __init__(self, robot_name):
        self.input_path = "/tmp/input_" + robot_name
        self.output_path = "/tmp/output_" + robot_name

        # Check if the file exists
        if os.path.exists(self.input_path):
            os.remove(self.input_path)
        if os.path.exists(self.output_path):
            os.remove(self.output_path)

        # Declare socket for monitor to process input
        # This socket is used for getting input from the monitor into the process running
        # or to the current program. This is how the user can control the execution since
        # this program can be assumed to run on the background
        self.sock_input = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock_input.settimeout(0.05)  # Set a timeout for socket operations
        self.sock_input.bind(self.input_path)
        self.sock_input.listen(1)  # Listen for one connection
        self.sock_input_connected = False
        self.sock_input_conn = None

        # Declare socket for process to monitor output
        # This socket takes output from both the process running and this program
        # and publishes it to the monitor. This allows the user to have some situational
        # awareness of what's going on.
        self.sock_output = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock_output.settimeout(0.05)  # Set a timeout for socket operations
        self.sock_output.bind(self.output_path)
        self.sock_output.listen(1)  # Listen for one connection
        self.sock_output_connected = False
        self.sock_output_conn = None

        # Declare event that will stop input thread
        self._stop_event = threading.Event()

    def __del__(self):
        loginfo("closing sockets!")
        self.sock_input.close()
        self.sock_output.close()

    def write_output_once(self, output):
        while not self.sock_output_connected:
            try:
                # If socket is not connected try to connect
                self.sock_output_conn, addr = self.sock_output.accept()
                self.sock_output_conn.setblocking(False)

                self.sock_output_connected = True
            except socket.timeout:
                continue

        try:
            if self.sock_output_connected:
                encoded_message = output.encode("ascii", errors="replace")
                for i in range(0, len(encoded_message), CHUNK_SIZE):
                    chunk = encoded_message[i : i + CHUNK_SIZE]
                    self.sock_output_conn.sendall(chunk)

        except (socket.error, BrokenPipeError):
            loginfo("Error sending data. Receiver may have disconnected.")
            self.sock_output_connected = False

    def thread_write_output(self, process):
        # loginfo("starting thread_write_output...")
        # Store cumulative output
        output_total = ""
        try:
            while not self._stop_event.is_set() and process.poll() is None:
                # Get output from process
                # loginfo("waiting for output")
                output = process.stdout.readline()
                if process.poll() is not None or self._stop_event.is_set():
                    break
                if output == "":
                    continue
                if output and not output.startswith("pos: x:"):
                    loginfo(f"writer received: {output}")
                    output_total += output

                try:
                    # If socket is not connected try to connect
                    if not self.sock_output_connected:
                        # loginfo("trying to connect")
                        self.sock_output_conn, addr = self.sock_output.accept()
                        self.sock_output_conn.setblocking(False)

                        self.sock_output_connected = True
                        encoded_message = output_total.encode("ascii", errors="replace")

                        for i in range(0, len(encoded_message), CHUNK_SIZE):
                            chunk = encoded_message[i : i + CHUNK_SIZE]
                            self.sock_output_conn.sendall(chunk)

                    # If socket is already connected, send output
                    elif self.sock_output_connected:
                        self.sock_output_conn.send(
                            output.encode("ascii", errors="replace")[:CHUNK_SIZE]
                        )
                except socket.timeout:
                    continue
                except (socket.error, BrokenPipeError):
                    loginfo("writer can't send data. Receiver may have disconnected.")
                    time.sleep(2)
                    self.sock_output_connected = False

        except Exception as e:
            loginfo(f"writer exiting on exception: {e}")
        # finally:
        #     # Save total output into a log
        #     loginfo(output_total)

    def read_input_once(self) -> str:
        while not (self.sock_input_connected or self._stop_event.is_set()):
            # loginfo("waiting for connection")
            try:
                self.sock_input_conn, addr = self.sock_input.accept()
                self.sock_input_conn.settimeout(
                    1
                )  # Set a timeout for socket operations
                self.sock_input_connected = True
                break
            except socket.timeout:
                continue
        while not self._stop_event.is_set():
            try:
                request = self.sock_input_conn.recv(CHUNK_SIZE).decode(
                    "ascii", errors="replace"
                )
                return request
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Connection was reset, set sock_input_connected to False
                self.sock_input_connected = False
                break
        return ""

    def thread_read_input(self, process):
        # loginfo("starting thread_read_input...")
        try:
            while not self._stop_event.is_set():
                while not (self.sock_input_connected or self._stop_event.is_set()):
                    # loginfo("waiting for connection")
                    try:
                        self.sock_input_conn, addr = self.sock_input.accept()
                        self.sock_input_conn.settimeout(1)
                        self.sock_input_connected = True
                        break
                    except socket.timeout:
                        continue

                while not self._stop_event.is_set():
                    try:
                        request = self.sock_input_conn.recv(CHUNK_SIZE).decode(
                            "ascii", errors="replace"
                        )
                        break
                    except socket.timeout:
                        continue
                    except ConnectionResetError:
                        # Connection was reset, set sock_input_connected to False
                        self.sock_input_connected = False
                        break
                if self._stop_event.is_set():
                    break

                # If broken pipe connect
                if not request:
                    break
                loginfo("reader sending: " + request)
                process.stdin.write(request + "\n")
                process.stdin.flush()
        except Exception as e:
            loginfo(f"reader exiting on exception: {e}")

    def send_command(self, command):
        loginfo(f"send_command: {command}")
        return_code = 1

        try:
            # Start the process
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            # Set the stdout stream to non-blocking
            fcntl.fcntl(process.stdout, fcntl.F_SETFL, os.O_NONBLOCK)

            # Start input and output threads
            input_thread = threading.Thread(
                target=self.thread_read_input, args=(process,)
            )
            input_thread.start()
            output_thread = threading.Thread(
                target=self.thread_write_output, args=(process,)
            )
            output_thread.start()

            # When the process finishes, te output thread automatically closes
            while output_thread.is_alive() and process.poll() is None:
                rospy.sleep(1)

        except Exception as e:
            loginfo(f"send_command exiting on exception: {e}")
        finally:
            # Forcefully stop the thread (not recommended)
            loginfo("send_command killing input thread...")
            self._stop_event.set()
            if output_thread.is_alive():
                output_thread.join()
            input_thread.join()

            # Reset for subsequent send_command() call
            self._stop_event.clear()

        loginfo(f"Check if child process {process.pid} running ")
        return_code = process.poll()
        if return_code is not None:
            loginfo(f"Child exited with exit status {return_code}")
            return return_code

        kill_time = 2.0
        loginfo(
            f"Child process {process.pid} is still running, kill -TERM then wait up to {kill_time} seconds"
        )
        process.terminate()
        try:
            return_code = process.wait(timeout=kill_time)
            loginfo(f"Child exited with exit status {return_code}")
            return return_code
        except subprocess.TimeoutExpired:
            loginfo(
                f"Child process {process.pid} is still running after {kill_time} seconds, kill -KILL"
            )
            process.send_signal(signal.SIGKILL)
            return_code = 1
            loginfo(f"Child killed, returning exit status {return_code}")
            return return_code

    def send_command_recursive(self, command):
        loginfo(f"Sending recursive command: {command}")

        exit_code = self.send_command(command)
        loginfo("send_command exit code " + str(exit_code))

        while exit_code != 0 and not rospy.is_shutdown():
            self.write_output_once(
                "Exit code non-zero: Do you want to repeat the send command? (yes/no/skip): \n"
            )
            repeat = self.read_input_once().lower()
            loginfo(f"user input: {repeat}")
            if repeat == "yes":
                exit_code = self.send_command_recursive(command)
                break
            if repeat == "no":
                break
            if repeat == "skip":
                exit_code = 0
                break
        return exit_code


# This class sends a command to the astrobee executor and waits to get a response
# Mostly used for short actions that should be immediate and require no feedback
# This method is needed on actions that run remotely and are not controlled by topics
class CommandExecutor:
    def __init__(self, ns: str):
        self.ns = ns
        loginfo(f"command topic: {self.ns}/command")
        # Declare guest science command publisher
        self.sub_ack = rospy.Subscriber(
            self.ns + "/mgt/ack", AckStamped, self.ack_callback
        )
        self.ack_needed = False
        self.ack_msg = None
        self.plan_status_needed = False
        self.sub_plan_status = rospy.Subscriber(
            self.ns + "/mgt/executive/plan_status",
            PlanStatusStamped,
            self.plan_status_callback,
        )
        self.plan_name = ""
        self.pub_command = rospy.Publisher(
            self.ns + "/command", CommandStamped, queue_size=5
        )
        while self.pub_command.get_num_connections() == 0 and not rospy.is_shutdown():
            loginfo(
                f"Waiting for an astrobee executive to subscribe to {self.ns}/command"
            )
            rospy.sleep(1)
        self.unique_cmd_id = ""

    def start_recording(self, bag_description):
        # Arg is bagfile name description
        arg1 = CommandArg()
        arg1.data_type = CommandArg.DATA_TYPE_STRING
        arg1.s = bag_description

        cmd = CommandStamped()
        cmd.header = Header(stamp=rospy.Time.now())
        cmd.cmd_name = CommandConstants.CMD_NAME_START_RECORDING
        cmd.cmd_id = "survey_manager" + str(rospy.Time.now().to_sec())
        self.unique_cmd_id = cmd.cmd_id
        cmd.cmd_src = "isaac fsw"
        cmd.cmd_origin = "isaac fsw"
        cmd.args = [arg1]

        # Publish the CommandStamped message
        loginfo(f"bag recording: Starting with description {bag_description}")
        result = self.publish_and_wait_response(cmd)
        return result

    def stop_recording(self):
        cmd = CommandStamped()
        cmd.header = Header(stamp=rospy.Time.now())
        cmd.cmd_name = CommandConstants.CMD_NAME_STOP_RECORDING
        cmd.cmd_id = "survey_manager" + str(rospy.Time.now().to_sec())
        self.unique_cmd_id = cmd.cmd_id
        cmd.cmd_src = "isaac fsw"
        cmd.cmd_origin = "isaac fsw"

        # Publish the CommandStamped message
        loginfo("bag recording: stopping recording")
        result = self.publish_and_wait_response(cmd)
        return result

    def change_exposure(self, val):
        # TBD
        loginfo("Change exposure to " + str(val))
        return 0

    def change_map(self, map_name):
        # TBD
        loginfo("Change map to " + map_name)
        return 0

    def ack_callback(self, msg):
        if self.ack_needed is True and msg.cmd_id == self.unique_cmd_id:
            self.ack_msg = msg
            self.ack_needed = False

    def plan_status_callback(self, msg):
        if self.plan_status_needed is True:
            loginfo(f"plan_name {self.plan_name}; msg name {msg.name}")
            if self.plan_name in msg.name:
                loginfo(f"In point {msg.point} status {msg.status.status}")
                if msg.status.status == 3:
                    self.plan_status_needed = False
            else:
                # Plan changed, and previous plan did not complete
                loginfo("Plan changed, exiting.")
                self.plan_status_needed = False

    def publish_and_wait_response(self, cmd):
        if rospy.is_shutdown():
            return 1
        # Publish the CommandStamped message
        self.ack_needed = True
        self.pub_command.publish(cmd)

        # Wait for ack
        counter = 0
        while counter < MAX_COUNTER:
            # got message
            if self.ack_needed is False:
                if self.ack_msg.completed_status.status == AckCompletedStatus.NOT:
                    loginfo("Command is being executed and has not completed.")
                    self.ack_needed = True
                elif self.ack_msg.completed_status.status == AckCompletedStatus.OK:
                    loginfo("Command completed successfully!")
                    return 0
                else:
                    loginfo("Command failed! Message: " + self.ack_msg.message)
                    return 1
            else:
                rospy.sleep(1)
                counter += 1
        return 1

    def wait_plan(self):
        if rospy.is_shutdown():
            return 1
        # Wait for ack
        counter = 0
        while counter < MAX_COUNTER:
            # got message
            if self.plan_status_needed is False:
                return 0
        return 1


def write_quick_pano(pano_path: pathlib.Path) -> pathlib.Path:
    """
    Return path to abbreviated version of `pano_path`. Generate the abbreviated version if it
    doesn't already exist.
    """
    # Example: "foo.txt" -> "foo-quick.txt"
    quick_path = pano_path.parent / (pano_path.stem + "-quick" + pano_path.suffix)
    if False:  # quick_path.exists():  # May be smarter to write every time
        loginfo(f"write_quick_pano: using existing quick target list {quick_path}")
        return quick_path

    # A bit arbitrary how to generate a "quick" version of a pano. Let's take
    # two frames.
    input_lines = pano_path.read_text(encoding="utf-8").splitlines()
    if len(input_lines) >= 39:
        # For our usual JEM panos, frames 37-38 should have pitch near zero and
        # require a bit less rotation than some others.
        output_lines = input_lines[37:39]
    else:
        # Must be some different pano type, best effort.
        output_lines = input_lines[:2]
    output_text = "".join([line + "\n" for line in output_lines])
    quick_path.write_text(output_text, encoding="utf-8")
    loginfo(f"write_quick_pano: wrote quick target list {quick_path}")
    return quick_path


def get_quick_stereo_survey(fplan_path: pathlib.Path) -> pathlib.Path:
    """
    Return path to abbreviated version of `fplan_path`, if it exists. These fplans are generated by
    manually editing the full fplan using the GDS plan editor.
    """
    # Example: "foo.fplan" -> "foo-quick.fplan"
    quick_path = fplan_path.parent / (fplan_path.stem + "-quick" + fplan_path.suffix)
    if quick_path.exists():
        loginfo(f"get_quick_stereo_survey: found quick survey {quick_path}")
        return quick_path

    loginfo(f"get_quick_stereo_survey: no quick survey available, using {fplan_path}")
    return fplan_path


@dataclass
class SurveyManagerExecutor:
    process_executor: ProcessExecutor
    command_executor: CommandExecutor
    config_static: YamlMapping
    fsw_ns_args: List[str]
    cmd_exec_ns: str

    def move(self, from_name: str, to_name: str) -> int:
        # Execute actual move
        move_args = self.config_static["bays_move"][to_name]
        cmd = (
            ["rosrun", "executive", "teleop_tool", "-remote", "-move"]
            + move_args
            + self.fsw_ns_args
        )
        exit_code = self.process_executor.send_command_recursive(cmd)
        if exit_code != 0:
            return exit_code

        # Change exposure if needed
        exposure_value = exposure_change(
            self.config_static,
            from_name,
            to_name,
        )
        if exposure_value != 0:
            exit_code = first_non_zero(
                exit_code, self.command_executor.change_exposure(exposure_value)
            )

        # Change map if needed
        map_name = map_change(self.config_static, from_name, to_name)
        if map_name != "":
            exit_code = first_non_zero(
                exit_code, self.command_executor.change_map(map_name)
            )

        return exit_code


def survey_manager_executor(args, run, config_static, process_executor, quick: bool):
    # Start ROS node
    rospy.init_node("survey_namager_cmd_" + args["robot"], anonymous=True)

    sim = False
    # Figure out robot name and whether we are in simulation or hardware
    current_robot = os.environ.get("ROBOTNAME")
    if not current_robot:
        loginfo("ROBOTNAME was not defined. Inferring robot from ROS topic /robot_name")
        # This is a latching messge so it shouldn't take long
        try:
            data = rospy.wait_for_message("/robot_name", String, timeout=5)
            current_robot = data.data.lower()
        except:
            current_robot = ""
        sim = True

    ns = []
    cmd_exec_ns = ""
    # If we're commanding a robot remotely
    if current_robot != args["robot"]:
        loginfo(
            f"We're commanding a namespaced robot! From '{current_robot}' to '{args['robot']}'"
        )
        ns = ["-ns", args["robot"]]
        # Command executor will add namespace for bridge forwarding
        cmd_exec_ns = "/" + args["robot"]
    command_executor = CommandExecutor(cmd_exec_ns)

    sm_exec = SurveyManagerExecutor(
        process_executor, command_executor, config_static, ns, cmd_exec_ns
    )

    # Initialize exit code
    exit_code = 0

    if args["type"] == "dock":
        # Avoid "Executive: Command failed with message: Dock goal failed with
        # response: Too far from dock"
        exit_code = sm_exec.move(args["from_name"], args["berth"])
        if exit_code != 0:
            return exit_code

        cmd = [
            "rosrun",
            "executive",
            "teleop_tool",
            "-dock",
            "-remote",
            "-berth",
            config_static["berth"][args["berth"]],
        ]
        cmd.extend(ns)

        exit_code = first_non_zero(
            exit_code, process_executor.send_command_recursive(cmd)
        )

    elif args["type"] == "undock":
        cmd = ["rosrun", "executive", "teleop_tool", "-undock", "-remote"]
        cmd.extend(ns)

        exit_code = first_non_zero(
            exit_code, process_executor.send_command_recursive(cmd)
        )

    elif args["type"] == "move":
        exit_code = sm_exec.move(args["from_name"], args["to_name"])

    elif args["type"] == "panorama":
        exit_code = first_non_zero(
            exit_code,
            command_executor.start_recording(
                "pano_" + args["location_name"] + "_" + run
            ),
        )
        if exit_code != 0:
            loginfo(
                "panorama: Failed to start recording, no point in starting the panorama"
            )
            return exit_code

        pano_path = pathlib.Path("resources") / pathlib.Path(
            config_static["bays_pano"][args["location_name"]]
        )
        if quick:
            inspection_path = pathlib.Path(rospkg.RosPack().get_path("inspection"))
            pano_path = write_quick_pano(inspection_path / pano_path).relative_to(
                inspection_path
            )

        cmd = [
            "rosrun",
            "inspection",
            "inspection_tool",
            "-geometry",
            "-geometry_poses",
            "/" + str(pano_path),
            "-remote",
        ]
        cmd.extend(ns)

        exit_code = first_non_zero(
            exit_code, process_executor.send_command_recursive(cmd)
        )
        loginfo("STOP RECORDING")
        exit_code = first_non_zero(exit_code, command_executor.stop_recording())

        # Sometimes the plan has a robot idle for an extended period after
        # completing a panorama. By default, when the inspection tool finishes
        # a panorama, the robot will be left in the pose it had for the final pano
        # frame, which might not be ideal for localization. This final move should
        # ensure the robot ends in the preferred attitude.
        exit_code = first_non_zero(
            exit_code, sm_exec.move(args["location_name"], args["location_name"])
        )

    elif args["type"] == "stereo":
        exit_code = first_non_zero(
            exit_code,
            command_executor.start_recording(
                "stereo_" + os.path.basename(args["fplan"]) + "_" + run
            ),
        )
        if exit_code != 0:
            loginfo(
                "stereo: Failed to start recording, no point in starting the stereo"
            )
            return exit_code

        # This starts the plan
        plan_path = pathlib.Path(get_ops_plan_path())
        fplan_path = plan_path / pathlib.Path(args["fplan"] + ".fplan")

        if quick:
            fplan_path = get_quick_stereo_survey(fplan_path)

        command_executor.plan_status_needed = True
        command_executor.plan_name = fplan_path.stem


        cmd = [
            "rosrun",
            "executive",
            "plan_pub",
            "-compression",
            "deflate",
            str(fplan_path),
            "-remote",
        ]
        cmd.extend(ns)

        exit_code = first_non_zero(
            exit_code, process_executor.send_command_recursive(cmd)
        )

        if exit_code == 0:
            exit_code = first_non_zero(exit_code, command_executor.wait_plan())

        exit_code = first_non_zero(exit_code, command_executor.stop_recording())

        # Ensure robot ends in preferred attitude for its bay.
        exit_code = first_non_zero(
            exit_code, sm_exec.move(args["base_name"], args["base_name"])
        )

    return exit_code


def survey_manager_executor_recursive(
    command_names, run_number, config_static, process_executor, quick: bool
):
    exit_code = survey_manager_executor(
        command_names, f"run{run_number}", config_static, process_executor, quick
    )

    while exit_code != 0 and not rospy.is_shutdown():
        process_executor.write_output_once(
            "Exit code non-zero: Do you want to repeat the survey? (yes/no/skip): \n"
        )
        repeat = process_executor.read_input_once().lower()

        if repeat == "yes":
            run_number += 1
            exit_code = survey_manager_executor_recursive(
                command_names, run_number, config_static, process_executor, quick
            )
            break
        if repeat == "no":
            break
        if repeat == "skip":
            exit_code = 0
            break

    return exit_code


def command_astrobee(
    action_args, config_static_paths: List[pathlib.Path], quick: bool
) -> int:
    # Read the static configs that convert constants to values
    config_static: YamlMapping = {}
    for config_static_path in config_static_paths:
        loginfo(f"reading config: {config_static_path}")
        yaml_dict = load_yaml(config_static_path)
        for key, value in yaml_dict.items():
            if key not in config_static:
                config_static[key] = value
            elif isinstance(value, dict):  # Merge nested dictionaries
                config_static[key].update(value)
            elif isinstance(value, list):  # Extend lists
                config_static[key].extend(value)
            else:  # Overwrite scalar values
                config_static[key] = value

    pddl_action = f"({' '.join(action_args)})"
    args = yaml_action_from_pddl(pddl_action, config_static)
    global INFO_CONTEXT
    INFO_CONTEXT = f"command_astrobee {pddl_action}"

    process_executor = ProcessExecutor(args["robot"])

    exit_code = survey_manager_executor_recursive(
        args, 1, config_static, process_executor, quick
    )

    loginfo(f"Finished plan action with code {exit_code}")
    return exit_code


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter):
    pass


def main():
    default_config_paths = [
        os.path.join(
            rospkg.RosPack().get_path("survey_planner"), "data/jem_survey_static.yaml"
        ),
        os.path.join(
            rospkg.RosPack().get_path("survey_planner"),
            "data/granite_survey_static.yaml",
        ),
    ]

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )

    parser.add_argument(
        "action_args",
        nargs="*",
        help="PDDL action name and its arguments",
    )
    parser.add_argument(
        "--config_static",
        help="Path to input static problem config YAML (module geometry, available stereo surveys, etc.)",
        type=pathlib.Path,
        nargs="+",
        default=[pathlib.Path(path) for path in default_config_paths],
    )
    parser.add_argument(
        "--quick",
        help="use quick versions of longer actions",
        action="store_true",
        default=False,
    )
    args = parser.parse_args()

    return command_astrobee(args.action_args, args.config_static, args.quick)


if __name__ == "__main__":
    sys.exit(main())
