#!/usr/bin/env python
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

import fnmatch
import importlib
import os

num_errors = 0


def get_cpplint_path():
    return os.path.dirname(os.path.realpath(__file__)) + "/cpplint.py"


def get_repo_path():
    return os.path.realpath(os.path.dirname(os.path.realpath(__file__)) + "/../..")


def run_cpplint(filename, cpplint_path):
    cpplint = importlib.machinery.SourceFileLoader(
        "cpplint", cpplint_path
    ).load_module()
    cpplint._cpplint_state.ResetErrorCounts()
    cpplint.print_stdout = False
    cpplint._line_length = 120
    cpplint.output = []
    try:
        index = filename.split("/").index("include")
        cpplint._root = "/".join(filename.split("/")[: index + 1])
    except ValueError:
        pass
    cpplint.ProcessFile(filename, cpplint._cpplint_state.verbose_level)
    return cpplint.output


def print_objection():
    print("Code formatting errors were found.")
    print("==================================")


def main():
    num_errors = 0

    cpplint_path = get_cpplint_path()
    repo_path = get_repo_path()

    # Lets look for source files and headers in our repo
    for root, dirnames, filenames in os.walk(get_repo_path()):
        if (
            "Software" in root
            or "external" in root
            or "submodules" in root
            or "debian" in root
            or "build" in root
        ):
            continue
        for filename in filenames:
            if "TinyEXIF" or "happly" in filename:
                continue
            if not filename.endswith(
                (
                    ".cpp",
                    ".cc",
                    ".h",
                    ".hpp",
                    ".cc.in",
                    ".c.in",
                    ".h.in",
                    ".hpp.in",
                    ".cxx",
                    ".hxx",
                )
            ):
                continue
            output = run_cpplint(
                (root + "/" + filename).replace(get_repo_path() + "/", ""), cpplint_path
            )
            # Print an objection at first sight of errors
            if num_errors == 0 and len(output) > 0:
                print_objection()

            num_errors += len(output)
            for error in output:
                print(
                    (
                        "%s:%s: %s"
                        % (
                            (root + "/" + filename).replace(get_repo_path() + "/", ""),
                            str(error[0]),
                            error[1],
                        )
                    )
                )

    print(("=" * 50))
    if num_errors > 0:
        print(("  You have %d lint errors." % num_errors))
    elif num_errors == 0:
        print("  Code adheres to style guidelines.")

    exit(num_errors)


if __name__ == "__main__":
    main()
