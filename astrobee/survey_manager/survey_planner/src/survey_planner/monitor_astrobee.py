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
import time  # Add time module for waiting

# Constants
CHUNK_SIZE = 1024


def thread_write_input(sock_input):
    print("Start thread_write_input")
    try:
        while True:
            user_input = input()
            print("user input: " + user_input)
            if user_input.lower().strip() == "exit":
                break
            sock_input.send(user_input.encode("ascii", errors="replace")[:CHUNK_SIZE])
    except Exception as e:
        print("Exception in thread_write_input:", e)


def thread_read_output(sock_output):
    print("Start thread_read_output")
    try:
        while True:
            request = sock_output.recv(CHUNK_SIZE)
            request = request.decode("ascii", errors="replace")
            print(request, end="")
    except Exception as e:
        print("Exception in thread_read_output:", e)


def survey_monitor(robot_name):
    input_path = "/tmp/input_" + robot_name
    output_path = "/tmp/output_" + robot_name

    while True:
        try:
            # Attempt to connect to the server
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

            # Wait for the threads to finish
            input_thread.join()
            output_thread.join()

            # Close the sockets
            sock_client_input.close()
            sock_client_output.close()

            # If the threads finish, break out of the loop
            break

        except ConnectionRefusedError:
            print("Connection refused. Retrying in 5 seconds...")
            time.sleep(5)
        except BrokenPipeError:
            print("Broken pipe detected. Restarting connection...")
            # If broken pipe occurs, close the sockets
            if "sock_client_input" in locals():
                sock_client_input.close()
            if "sock_client_output" in locals():
                sock_client_output.close()
        except Exception as e:
            print("Exception:", e)
            continue


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
