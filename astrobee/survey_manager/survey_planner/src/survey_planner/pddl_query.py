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
Answers a query about the PDDL domain.

Available query types:
- action_duration <action_name>

Example:
$ pddl_query.py --domain=domain_survey.pddl action_duration panorama
"""

import argparse
import enum
import pathlib
from typing import List

from survey_planner.plan_survey import get_action_durations, parse_pddl
from survey_planner.problem_generator import PDDL_DIR


@enum.unique
class QueryType(enum.Enum):
    "Represents available query types."
    ACTION_DURATION = "action_duration"


QUERY_TYPES = [qtype.value for qtype in QueryType]


def pddl_query(
    domain_path: pathlib.Path,
    query_type: QueryType,
    query_args: List[str],
) -> None:
    """
    The main function that answers a query.
    """
    if query_type == QueryType.ACTION_DURATION:
        if len(query_args) != 1:
            raise ValueError(
                "action_duration query requires exactly 1 argument (the action name)"
            )
        [action_name] = query_args
        durations = get_action_durations(parse_pddl(domain_path))
        if action_name not in durations:
            raise KeyError(
                f"Expected action_name in {list(durations.keys())}, got '{action_name}'"
            )
        print(durations[action_name])
    else:
        assert False, "Never reach this point."


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    "Custom formatter for argparse that combines mixins."


def main() -> None:
    "Parse arguments and invoke pddl_query()."
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "--domain",
        help="path for input PDDL domain",
        type=pathlib.Path,
        default=PDDL_DIR / "domain_survey.pddl",
    )
    parser.add_argument(
        "query_arg",
        help="query type and arguments",
        nargs="+",
    )
    args = parser.parse_args()

    try:
        query_type = QueryType(args.query_arg[0])
    except ValueError:
        parser.error(f"Expected query type in {QUERY_TYPES}, got '{args.query_arg[0]}'")

    pddl_query(args.domain, query_type, args.query_arg[1:])


if __name__ == "__main__":
    main()
