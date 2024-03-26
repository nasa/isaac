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

# Declare event that will stop input thread
stop_event = threading.Event()


def thread_write_input(input_path):
    print("Start thread_write_input")

    user_input = ""
    while not stop_event.is_set():
        try:
            # Attempt to connect to the server
            sock_client_input = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock_client_input.connect(input_path)
        except ConnectionRefusedError:
            # print("Input Connection refused. Retrying in 5 seconds...")
            time.sleep(1)
            continue

        try:
            while not stop_event.is_set():
                if user_input == "":
                    user_input = input()
                # print("user input: " + user_input)
                if user_input.lower().strip() == "exit":
                    stop_event.set()
                if stop_event.is_set():
                    break
                sock_client_input.send(
                    user_input.encode("ascii", errors="replace")[:CHUNK_SIZE]
                )
                user_input = ""
        except Exception as e:
            # print("Exception in thread_write_input:", e)
            pass
        # Close the sockets
        print("Close input socket")
        sock_client_input.close()


def thread_read_output(output_path):
    print("Start thread_read_output")

    while not stop_event.is_set():
        try:
            # Attempt to connect to the server
            sock_client_output = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock_client_output.settimeout(1)  # Set timeout to 1 second
            sock_client_output.connect(output_path)

            while not stop_event.is_set():
                try:
                    data = sock_client_output.recv(CHUNK_SIZE)
                    if not data:
                        print("Server disconnected")
                        break
                    print(data.decode("ascii", errors="replace"), end="")
                except socket.timeout:
                    continue  # Timeout reached, check stop event and try again
                except Exception as e:
                    break  # Handle other exceptions or break
        except ConnectionRefusedError:
            time.sleep(5)  # Wait a bit before retrying to connect
            continue
        except socket.timeout:
            continue
        print("Close output socket")
        sock_client_output.close()


def survey_monitor(robot_name):
    try:
        input_path = "/tmp/input_" + robot_name
        output_path = "/tmp/output_" + robot_name

        while not (os.path.exists(input_path) and os.path.exists(output_path)):
            print("Files don't exist yet. Waiting for 5 seconds...")
            time.sleep(5)
            continue

        # Start input and output threads
        input_thread = threading.Thread(target=thread_write_input, args=(input_path,))
        output_thread = threading.Thread(target=thread_read_output, args=(output_path,))
        input_thread.start()
        output_thread.start()

        # Wait for the threads to finish
        input_thread.join()
        output_thread.join()

    except KeyboardInterrupt:
        print("\nGracefully shutting down, Press Enter...")
        stop_event.set()
        input_thread.join()
        output_thread.join()
        print("Shutdown complete.")


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
