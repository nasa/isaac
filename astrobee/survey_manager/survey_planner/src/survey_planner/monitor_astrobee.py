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
import socket
import sys
import threading

# Constants
CHUNK_SIZE = 1024


def thread_write_input(sock_input):
    # print("starting thread_write_input...")
    try:
        while True:
            # Get user input dynamically
            user_input = input()
            print("user input: " + user_input)
            # Check if the user wants to exit
            if user_input.lower().strip() == "exit":
                break
            sock_input.send(user_input.encode("ascii", errors="replace")[:CHUNK_SIZE])

    except:
        print("exit output")


def thread_read_output(sock_output):
    # print("starting thread_read_output...")
    try:
        while True:
            request = sock_output.recv(CHUNK_SIZE)
            request = request.decode("ascii", errors="replace")  # convert bytes to str

            print(request, end="")
    except:
        print("exit input")


def survey_monitor(robot_name):
    input_path = "/tmp/input_" + robot_name
    output_path = "/tmp/output_" + robot_name

    sock_client_input = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock_client_input.connect(input_path)
    sock_client_output = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock_client_output.connect(output_path)

    # Start input and output threads
    input_thread = threading.Thread(
        target=thread_write_input, args=(sock_client_input,)
    )
    input_thread.start()
    output_thread = threading.Thread(
        target=thread_read_output, args=(sock_client_output,)
    )
    output_thread.start()

    # Wait for the thread
    input_thread.join()
    output_thread.join()

    # Close the sockets
    sock_client_input.close()
    sock_client_output.close()


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )

    parser.add_argument(
        "robot_name",
        help="Name of the robot that needs monitoring.",
    )
    args = parser.parse_args()

    survey_monitor(args.robot_name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
