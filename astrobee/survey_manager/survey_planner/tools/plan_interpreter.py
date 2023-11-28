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
Provides an example of how to interpret the actions in a survey domain plan, dropping the irrelevant
arguments, mapping the positional arguments back to the names used in the dynamic config, and
looking up extra info that's not needed by the planner but is important for execution (e.g.,
coordinates of named locations, fplan filenames of stereo surveys).

For testing, you can call this from the command line on a complete sample output plan.

For use during execution, if you are receiving one action at a time from an executor, you may
prefer to import this module and call the yaml_action_from_pddl() function directly.
"""

import argparse
import pathlib
import re
import sys
from typing import Any, Dict, List

import yaml

ACTION_TYPE_OPTIONS = ("dock", "undock", "move", "panorama", "stereo")


# Type alias
YamlMapping = Dict[str, Any]
FloatStr = str  # String representation of a floating-point value

# POPF plan output weirdly mixes its random debugging output with the actual plan actions that it
# outputs in a standard format. (Maybe there's some way to suppress the debug info?) Anyway, this
# regex works nicely to ignore the debug info and parse the fields of the plan actions we care
# about. It might need fine-tuning if we use other planners.
PLAN_ACTION_REGEX = re.compile(
    r"^(?P<start_time_seconds>\d+(\.\d*)?): (?P<action>\(.*?\))\s+\[(?P<duration_seconds>\d+(\.\d*)?)\]\s*$"
)


def load_yaml(yaml_path: pathlib.Path) -> YamlMapping:
    """
    Return the YAML parse result for the file at `yaml_path`.
    """
    with yaml_path.open(encoding="utf-8") as yaml_stream:
        return yaml.safe_load(yaml_stream)


class PlanAction:
    """
    Class representing one entry in the output plan sequence of a PDDL planner.
    """

    def __init__(self, start_time_seconds: FloatStr, action: str, duration_seconds: FloatStr):
        self.start_time_seconds = start_time_seconds
        self.action = action
        self.duration_seconds = duration_seconds

    def __repr__(self):
        return f"{self.start_time_seconds}: {self.action} [{self.duration_seconds}]"


def yaml_action_from_pddl(action: str, static_config: YamlMapping) -> YamlMapping:
    """
    Return a YamlMapping representation of `action`. This is the only place
    we really need domain-specific logic.
    """
    action_args = action[1:-1].split()
    action_type = action_args[0]
    assert (
        action_type in ACTION_TYPE_OPTIONS
    ), f"Expected action type in {ACTION_TYPE_OPTIONS}, got {action_type}"

    if action_type == "dock":
        robot, _from_bay, to_berth = action_args[1:]
        # Can discard from_bay
        return {"type": "dock", "robot": robot, "berth": to_berth}

    if action_type == "undock":
        robot, _from_berth, _to_bay, _check1, _check2 = action_args[1:]
        # Can discard from_berth, to_bay, check1, check2
        return {"type": "undock", "robot": robot}

    if action_type == "move":
        robot, _from_bay, to_bay, _check_bay = action_args[1:]
        # Can discard from_bay, check_bay. Look up coordinates for to_bay.
        return {
            "type": "move",
            "robot": robot,
            "to_name": to_bay,
            "to_pos": static_config["bays"][to_bay],
        }

    if action_type == "panorama":
        robot, _order, location, run_name = action_args[1:]
        run_number = int(run_name[-1])
        # Can discard order. Look up coordinates for location.
        return {
            "type": "panorama",
            "robot": robot,
            "location_name": location,
            "location_pos": static_config["bays"][location],
            "run": run_number,
        }

    if action_type == "stereo":
        robot, _order, base, bound, _check1, _check2, run_name = action_args[1:]
        run_number = int(run_name[-1])
        # Use base and bound to look up trajectory.
        traj_matches = [
            traj
            for traj in static_config["stereo"].values()
            if traj["base_location"] == base and traj["bound_location"] == bound
        ]
        assert (
            len(traj_matches) == 1
        ), f"Expected exactly 1 matching stereo trajectory with base {base} and bound {bound}, got {len(traj_matches)}"
        fplan = traj_matches[0]["fplan"]
        # Can discard order, base, bound, check1, check2.
        return {"type": "stereo", "robot": robot, "fplan": fplan, "run": run_number}

    assert False, "Never reach this point."
    return {}  # Make pylint happy


def yaml_plan_action_from_pddl(
    plan_action: PlanAction, static_config: YamlMapping
) -> YamlMapping:
    """
    Return a YamlMapping representation of `plan_action`.
    """
    return {
        "start_time_seconds": plan_action.start_time_seconds,
        "action": yaml_action_from_pddl(plan_action.action, static_config),
        "duration_seconds": plan_action.duration_seconds,
    }


def parse_plan(plan_path: pathlib.Path) -> List[PlanAction]:
    """
    Return a list of PlanActions read from the PDDL plan at `plan_path`.
    """
    actions = []
    with plan_path.open(encoding="utf-8") as plan_stream:
        for plan_line in plan_stream:
            match = PLAN_ACTION_REGEX.search(plan_line)
            if match:
                start_time_seconds = match.group("start_time_seconds")
                action = match.group("action")
                duration_seconds = match.group("duration_seconds")

                # Raise an exception if these values are not the string representation of a
                # floating-point value. However, we are storing the original string representation
                # rather than storing the float to avoid any weird issues with the values getting
                # mangled due to floating-point precision and formatting.
                float(start_time_seconds)
                float(duration_seconds)

                actions.append(PlanAction(start_time_seconds, action, duration_seconds))
    return actions


class NoAliasDumper(
    yaml.SafeDumper
):  # pylint: disable=too-many-ancestors  # It only has 1, so (?)
    """
    Configures yaml dump output to avoid using confusing aliases.
    """

    def ignore_aliases(self, data):
        return True


def plan_interpreter(
    config_static_path: pathlib.Path, plan_path: pathlib.Path, output_path: pathlib.Path
) -> None:
    """
    The main function that interprets an entire plan file.

    However, if you are receiving one action at a time from an executor, you may prefer to import
    this module and call yaml_action_from_pddl() directly.
    """
    config_static = load_yaml(config_static_path)
    pddl_actions = parse_plan(plan_path)
    yaml_actions = [
        yaml_plan_action_from_pddl(plan_action, config_static)
        for plan_action in pddl_actions
    ]
    with output_path.open("w", encoding="utf-8") as output_stream:
        yaml.dump(yaml_actions, output_stream, Dumper=NoAliasDumper, sort_keys=False)
    print(f"Wrote to {output_path}", file=sys.stderr)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "--config-static",
        help="Path to input static problem config YAML (module geometry, available stereo surveys, etc.)",
        type=pathlib.Path,
        default="jem_survey_static.yaml",
    )
    parser.add_argument(
        "--plan",
        help="Path to input plan generated by PDDL planner (parser currently tuned for POPF idiosyncrasies)",
        type=pathlib.Path,
        default="sample_output_plan.txt",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Path for output converted to YAML format",
        type=pathlib.Path,
        default="sample_output_plan.yaml",
    )
    args = parser.parse_args()

    plan_interpreter(
        config_static_path=args.config_static,
        plan_path=args.plan,
        output_path=args.output,
    )


if __name__ == "__main__":
    main()
