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
import os
import pathlib
import select
import socket
import subprocess
import sys
import threading
from typing import Any, Dict, List

import rospkg
import rospy
import yaml
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
from survey_planner.problem_generator import load_yaml, yaml_action_from_pddl

# Constants
MAX_COUNTER = 10
CHUNK_SIZE = 1024


def exposure_change(config_static, bay_origin, bay_destination):
    # Going to JEM
    if bay_origin == "nod2_hatch_to_jem" and bay_destination == "jem_hatch_from_nod2":
        print("CHANGING EXPOSURE TO JEM")
        return config_static["exposure"]["jem"]

    # Going  to NOD2
    if (
        bay_origin == "jem_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_jem"
        or bay_origin == "usl_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_usl"
    ):
        print("CHANGING EXPOSURE TO NOD2")
        return config_static["exposure"]["nod2"]

    # Going to USL
    if bay_origin == "nod2_hatch_to_usl" and bay_destination == "usl_hatch_from_nod2":
        return config_static["exposure"]["usl"]

    return 0


def map_change(config_static, bay_origin, bay_destination):
    # Going to JEM
    if bay_origin == "nod2_hatch_to_jem" and bay_destination == "jem_hatch_from_nod2":
        print("CHANGING MAP TO JEM")
        return config_static["maps"]["jem"]
    # Going  to NOD2
    if (
        bay_origin == "jem_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_jem"
        or bay_origin == "usl_hatch_to_nod2"
        and bay_destination == "nod2_hatch_from_usl"
    ):
        print("CHANGING MAP TO NOD2")
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
        return os.path.join(astrobee_ops_path, "gds/plans")

    # Check if the symlink ~/gds/latest/ControlStationConfig/IssWorld exists
    symlink_path = os.path.expanduser("~/gds/latest/ControlStationConfig/IssWorld")
    if os.path.islink(symlink_path):
        # Get the target of the symlink
        target_path = os.path.realpath(symlink_path)
        # Construct the relative path ../../plans
        relative_path = os.path.join(target_path, "../../plans")
        return relative_path

    # Return None if none of the conditions are met
    return None


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

        # Declare socket for process input
        self.sock_input = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock_input.settimeout(0.05)  # Set a timeout for socket operations
        self.sock_input.bind(self.input_path)
        self.sock_input.listen(1)  # Listen for one connection
        self.sock_input_connected = False

        # Declare socket for process output
        self.sock_output = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock_output.settimeout(0.05)  # Set a timeout for socket operations
        self.sock_output.bind(self.output_path)
        self.sock_output.listen(1)  # Listen for one connection
        self.sock_output_connected = False

        # Declare event that will stop input thread
        self._stop_event = threading.Event()

    def __del__(self):
        print("closing sockets!")
        self.sock_input.close()
        self.sock_output.close()

    def thread_write_output(self, process):
        # print("starting thread_write_output...")
        # Store cumulative output
        output_total = ""
        try:
            while True:
                # Get output from process
                # print("waiting for output")
                output = process.stdout.readline()
                if (
                    output == "" and process.poll() is not None
                ) or self._stop_event.is_set():
                    break
                if output and not output.startswith("pos: x:"):
                    rospy.loginfo(output)
                    output_total += output

                try:
                    # If socket is not connected try to connect
                    if not self.sock_output_connected:
                        # print("trying to connect")
                        conn, addr = self.sock_output.accept()
                        conn.setblocking(False)

                        self.sock_output_connected = True
                        encoded_message = output_total.encode("ascii", errors="replace")

                        for i in range(0, len(encoded_message), CHUNK_SIZE):
                            chunk = encoded_message[i : i + CHUNK_SIZE]
                            conn.sendall(chunk)

                    # If socket is already connected, send output
                    elif self.sock_output_connected:
                        conn.send(output.encode("ascii", errors="replace")[:CHUNK_SIZE])
                except socket.timeout:
                    continue
                except (socket.error, BrokenPipeError):
                    print("Error sending data. Receiver may have disconnected.")
                    self.sock_output_connected = False

        except Exception as e:
            print("exit output:")
            print(e)
        # finally:
        #     # Save total output into a log
        #     rospy.loginfo(output_total)

    def thread_read_input(self, process):
        # print("starting thread_read_input...")
        try:
            while True:
                while not self._stop_event.is_set():
                    # print("waiting for connection")
                    try:
                        client_socket, client_address = self.sock_input.accept()
                        break
                    except socket.timeout:
                        continue
                if self._stop_event.is_set():
                    break
                client_socket.settimeout(1)  # Set a timeout for socket operations

                while True:
                    # print("accepted connection:")
                    print(client_address)

                    while not self._stop_event.is_set():
                        # print("waiting to receive")
                        try:
                            request = client_socket.recv(CHUNK_SIZE).decode(
                                "ascii", errors="replace"
                            )
                            break
                        except socket.timeout:
                            continue
                    if self._stop_event.is_set():
                        break

                    # If broken pipe connect again
                    if not request:
                        break
                    print("got: " + request)

                    print(request)
                    process.stdin.write(request + "\n")
                    process.stdin.flush()
        except Exception as e:
            print("exit input:")
            print(e)

    def send_command(self, command):
        print(command)
        return_code = 1

        try:
            # Start the process
            process = subprocess.Popen(
                command,
                shell=True,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            # Start input and output threads
            input_thread = threading.Thread(
                target=self.thread_read_input, args=(process,)
            )
            input_thread.start()
            output_thread = threading.Thread(
                target=self.thread_write_output, args=(process,)
            )
            output_thread.start()

            while output_thread.is_alive():
                rospy.sleep(1)
            output_thread.join()
            # Get the return code of the process
            return_code = process.poll()

        except Exception as e:
            print("exit main:")
            print(e)
            # Get the return code of the process
            process.kill()
        finally:
            # Forcefully stop the thread (not recommended)
            print("Killing input thread...")
            self._stop_event.set()
            input_thread.join()
            if output_thread.is_alive():
                output_thread.join()

            # Get the final exit code
            return return_code

    def send_command_recursive(self, command):
        print("Sending recursive command")

        exit_code = self.send_command(command)
        print("Exit code " + str(exit_code))

        if exit_code != 0 and not rospy.is_shutdown():
            repeat = input("Do you want to repeat the command? (yes/no): ").lower()
            print(repeat)
            if repeat == "yes":
                exit_code = exit_code = self.send_command_recursive(command)
        return exit_code


# This class sends a command to the astrobee executor and waits to get a response
# Mostly used for short actions that should be immediate and require no feedback
# This method is needed on actions that run remotely and are not controlled by topics
class CommandExecutor:
    def __init__(self, ns):
        self.ns = ns
        rospy.loginfo(self.ns + "/command")
        # Declare guest science command publisher
        self.sub_ack = rospy.Subscriber(
            self.ns + "/mgt/ack", AckStamped, self.ack_callback
        )
        self.ack_needed = False
        self.sub_plan_status = rospy.Subscriber(
            self.ns + "/mgt/executive/plan_status",
            PlanStatusStamped,
            self.plan_status_callback,
        )
        self.plan_status_needed = False
        self.plan_name = ""
        self.pub_command = rospy.Publisher(
            self.ns + "/command", CommandStamped, queue_size=5
        )
        while self.pub_command.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscriber to connect")
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
        result = self.publish_and_wait_response(cmd)
        return result

    def change_exposure(self, val):
        # TBD
        rospy.loginfo("Change exposure to " + str(val))
        return 0

    def change_map(self, map_name):
        # TBD
        rospy.loginfo("Change map to " + map_name)
        return 0

    def ack_callback(self, msg):
        if self.ack_needed == True and msg.cmd_id == self.unique_cmd_id:
            self.ack_msg = msg
            self.ack_needed = False

    def plan_status_callback(self, msg):
        if self.plan_status_needed == True:
            rospy.loginfo("plan_name" + self.plan_name + "; msg name " + msg.name)
            if self.plan_name in msg.name:
                rospy.loginfo(
                    "In point " + str(msg.point) + " status " + str(msg.status.status)
                )
                if msg.status.status == 3:
                    self.plan_status_needed = False
            else:
                # Plan changed, and previous plan did not complete
                rospy.loginfo("Plan changed, exiting.")
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
            if self.ack_needed == False:
                if self.ack_msg.completed_status.status == AckCompletedStatus.NOT:
                    rospy.loginfo("Command is being executed and has not completed.")
                    self.ack_needed = True
                elif self.ack_msg.completed_status.status == AckCompletedStatus.OK:
                    rospy.loginfo("Command completed successfully!")
                    return 0
                else:
                    rospy.loginfo("Command failed! Message: " + self.ack_msg.message)
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
            if self.plan_status_needed == False:
                return 0
        return 1


def survey_manager_executor(command_names, run, config_static_path: pathlib.Path):
    # Read the static configs that convert constants to values
    config_static = load_yaml(config_static_path)

    args = yaml_action_from_pddl(f"[{' '.join(command_names)}]", config_static)

    # Start ROS node
    rospy.init_node("survey_namager_cmd_" + args["robot"], anonymous=True)

    sim = False
    # Figure out robot name and whether we are in simulation or hardware
    current_robot = os.environ.get("ROBOTNAME")
    if not current_robot:
        rospy.loginfo("ROBOTNAME not defined. Let's get the robotname using the topic")
        # This is a latching messge so it shouldn't take long
        try:
            data = rospy.wait_for_message("/robot_name", String, timeout=5)
            current_robot = data.data.lower()
        except:
            current_robot = ""
        sim = True

    ns = ""
    # If we're commanding a robot remotely
    if current_robot != args["robot"]:
        rospy.loginfo("We're commanding a namespaced robot!")
        ns = " -remote -ns " + args["robot"]
        # Command executor will add namespace for bridge forwarding
        command_executor = CommandExecutor("/" + args["robot"])
    else:
        command_executor = CommandExecutor("/")
    process_executor = ProcessExecutor(args["robot"])

    # Initialize exit code
    exit_code = 0

    if args["type"] == "dock":
        exit_code += process_executor.send_command_recursive(
            "rosrun executive teleop_tool -dock"
            + ns
            + " -berth "
            + config_static["berth"][args["berth"]]
        )

    elif args["type"] == "undock":
        exit_code += process_executor.send_command_recursive(
            "rosrun executive teleop_tool -undock" + ns
        )

    elif args["type"] == "move":
        exit_code += process_executor.send_command_recursive(
            "rosrun executive teleop_tool -move "
            + config_static["bays_move"][args["to_name"]]
            + ns
        )
        # Change exposure if needed
        exposure_value = exposure_change(
            config_static, args["from_name"], args["to_name"]
        )
        if exposure_value != 0:
            exit_code += command_executor.change_exposure(exposure_value)
        # Change map if needed
        map_name = map_change(config_static, args["from_name"], args["to_name"])
        if map_name != "":
            exit_code += command_executor.change_map(map_name)

    elif args["type"] == "panorama":
        exit_code += command_executor.start_recording(
            "pano_" + args["location_name"] + "_" + run
        )
        exit_code += process_executor.send_command_recursive(
            "rosrun inspection inspection_tool -geometry -geometry_poses /resources/"
            + config_static["bays_pano"][args["location_name"]]
            + ns
        )
        exit_code += command_executor.stop_recording()

    elif args["type"] == "stereo":
        exit_code += command_executor.start_recording(
            "stereo_" + os.path.basename(args["fplan"]) + "_" + run
        )
        # This starts the plan
        plan_path = get_ops_plan_path()

        command_executor.plan_status_needed = True
        command_executor.plan_name = os.path.basename(args["fplan"])
        exit_code += process_executor.send_command_recursive(
            "rosrun executive plan_pub "
            + os.path.join(plan_path, args["fplan"] + ".fplan")
            + ns
        )
        if exit_code == 0:
            exit_code += command_executor.wait_plan()
        exit_code += command_executor.stop_recording()

    return exit_code


def survey_manager_executor_recursive(
    command_names, run_number, config_static_path: pathlib.Path
):
    exit_code = survey_manager_executor(
        command_names, f"run{run_number}", config_static_path
    )

    if exit_code != 0:
        repeat = input("Do you want to repeat the survey? (yes/no/skip): ").lower()
        if repeat == "yes":
            run_number += 1
            exit_code = survey_manager_executor_recursive(
                command_names, run_number, config_static_path
            )
        if repeat == "skip":
            exit_code = 0

    return exit_code


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )

    parser.add_argument(
        "command_names",
        nargs="*",
        help="Prefixes for bagfiles to merge. Bags should all be in the current working directory.",
    )
    parser.add_argument(
        "--config_static",
        help="Path to input static problem config YAML (module geometry, available stereo surveys, etc.)",
        type=pathlib.Path,
        default=os.path.join(
            rospkg.RosPack().get_path("survey_planner"), "data/survey_static.yaml"
        ),
    )
    args = parser.parse_args()

    exit_code = survey_manager_executor_recursive(
        args.command_names, 1, args.config_static
    )

    print("Finished plan action with code " + str(exit_code))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
