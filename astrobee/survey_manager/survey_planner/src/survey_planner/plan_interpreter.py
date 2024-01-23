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
import collections
import pathlib
import re
import sys
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple, TypeVar

import numpy as np
import yaml
from matplotlib import collections as mc
from matplotlib import patches as mp
from matplotlib import pyplot as plt
from problem_generator import (
    DATA_DIR,
    get_stereo_traj,
    load_yaml,
    path_list,
    yaml_action_from_pddl,
)

DEFAULT_CONFIGS = [
    DATA_DIR / "survey_static.yaml",
    # Dynamic config not needed for interpreting the plan
]

COLORS = {"bumble": "#4080ffff", "honey": "#c0c080ff"}
ROBOTS = list(COLORS.keys())

# Type alias
YamlMapping = Dict[str, Any]
FloatStr = str  # String representation of a floating-point value
T = TypeVar("T")  # pylint: disable=invalid-name

# POPF plan output weirdly mixes its random debugging output with the actual plan actions that it
# outputs in a standard format. (Maybe there's some way to suppress the debug info?) Anyway, this
# regex works nicely to ignore the debug info and parse the fields of the plan actions we care
# about. It might need fine-tuning if we use other planners. Note: The regex also seems to work ok
# with OPTIC, which was derived from POPF.
PLAN_ACTION_REGEX = re.compile(
    r"^(?P<start_time_seconds>\d+(\.\d*)?): (?P<action>\(.*?\))\s+\[(?P<duration_seconds>\d+(\.\d*)?)\]\s*$"
)


class PlanAction:
    """
    Class representing one entry in the output plan sequence of a PDDL planner.
    """

    def __init__(
        self, start_time_seconds: FloatStr, action: str, duration_seconds: FloatStr
    ):
        self.start_time_seconds = start_time_seconds
        self.action = action
        self.duration_seconds = duration_seconds

    def __repr__(self):
        return f"{self.start_time_seconds}: {self.action} [{self.duration_seconds}]"


def yaml_plan_action_from_pddl(
    plan_action: PlanAction, static_config: YamlMapping
) -> Optional[YamlMapping]:
    """
    Return a YamlMapping representation of `plan_action`.
    """
    action = yaml_action_from_pddl(plan_action.action, static_config)
    if action is None:
        return None
    return {
        "start_time_seconds": plan_action.start_time_seconds,
        "action": action,
        "duration_seconds": plan_action.duration_seconds,
    }


def parse_plan(plan_path: pathlib.Path) -> List[PlanAction]:
    """
    Return a list of PlanActions read from the PDDL plan at `plan_path`.
    """
    actions: List[PlanAction] = []
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

                # One of our PDDL planners being evaluated is OPTIC, which is an anytime
                # planner. Its overall output potentially includes multiple plans (with increasing
                # quality) output during the course of a single planner run. We want to use only the
                # last plan.  An easy way to do that is to discard all previous actions if we see a
                # new action starting at the overall plan start time (t = 0).
                if float(start_time_seconds) == 0 and actions:
                    print(
                        "WARNING: Found the start of another plan; will use only the last plan",
                        file=sys.stderr,
                    )
                    actions = []

                actions.append(PlanAction(start_time_seconds, action, duration_seconds))
    return actions


def filter_none(elts: Iterable[Optional[T]]) -> List[T]:
    "Return `elts` filtered to remove `None` values."
    return [x for x in elts if x is not None]


class NoAliasDumper(
    yaml.SafeDumper
):  # pylint: disable=too-many-ancestors  # It only has 1, so (?)
    """
    Configures yaml dump output to avoid using confusing aliases.
    """

    def ignore_aliases(self, data):
        return True


def save_plan_yaml(output_path: pathlib.Path, yaml_actions: List[YamlMapping]) -> None:
    "Save plan `yaml_actions` to `output_path` in YAML format."
    with output_path.open("w", encoding="utf-8") as output_stream:
        yaml.dump(yaml_actions, output_stream, Dumper=NoAliasDumper, sort_keys=False)
    print(f"Wrote YAML plan to {output_path}", file=sys.stderr)


def plot_plan(plot_path: pathlib.Path, yaml_actions: List[YamlMapping]) -> None:
    "Save a plot image of plan `yaml_actions` to `plot_path`."
    # Type aliases
    TimePos = Tuple[float, int]
    PanoLine = Tuple[TimePos, TimePos]

    robot_time_pos: Mapping[str, List[TimePos]] = collections.defaultdict(list)
    robot_pano_lines: Mapping[str, List[PanoLine]] = collections.defaultdict(list)
    robot_stereo_rects: Mapping[str, List[mp.Rectangle]] = collections.defaultdict(list)

    for plan_action in yaml_actions:
        action = plan_action["action"]
        robot = action["robot"]
        start_time_seconds = float(plan_action["start_time_seconds"])
        duration_seconds = float(plan_action["duration_seconds"])
        end_time_seconds = start_time_seconds + duration_seconds
        if action["type"] == "move":
            from_bay = int(action["from_name"][-1])
            to_bay = int(action["to_name"][-1])
            robot_time_pos[robot] += [
                (start_time_seconds, from_bay),
                (end_time_seconds, to_bay),
            ]
        elif action["type"] == "undock":
            from_bay = 8
            to_bay = 7
            robot_time_pos[robot] += [
                (start_time_seconds, from_bay),
                (end_time_seconds, to_bay),
            ]
        elif action["type"] == "dock":
            from_bay = 7
            to_bay = 8
            robot_time_pos[robot] += [
                (start_time_seconds, from_bay),
                (end_time_seconds, to_bay),
            ]
        elif action["type"] == "panorama":
            bay_number = int(action["location_name"][-1])
            robot_pano_lines[robot].append(
                ((start_time_seconds, bay_number), (end_time_seconds, bay_number))
            )
        elif action["type"] == "stereo":
            base_bay = int(action["base_name"][-1])
            bound_bay = int(action["bound_name"][-1])
            bay1, bay2 = sorted((base_bay, bound_bay))
            padding = 0.5
            robot_stereo_rects[robot].append(
                mp.Rectangle(
                    xy=(start_time_seconds, bay1 - padding),
                    width=duration_seconds,
                    height=bay2 - bay1 + 2 * padding,
                )
            )

    fig, ax = plt.subplots()  # pylint: disable=invalid-name
    for robot in ROBOTS:
        time_pos = np.array(robot_time_pos[robot])
        plt.plot(time_pos[:, 0], time_pos[:, 1], "o-", label=robot, color=COLORS[robot])

        if robot_pano_lines[robot]:
            ax.add_collection(
                mc.LineCollection(
                    robot_pano_lines[robot], color=COLORS[robot], linewidth=10
                )
            )
        if robot_stereo_rects[robot]:
            ax.add_collection(
                mc.PatchCollection(
                    robot_stereo_rects[robot], facecolor=COLORS[robot], alpha=0.25
                )
            )

    plt.xlabel("Time (s)")
    plt.ylabel("Bay number (dock = 8)")
    plt.grid("both")
    plt.legend()
    fig.set_size_inches((24, 4))
    plt.savefig(plot_path)
    print(f"Wrote plot to {plot_path}", file=sys.stderr)


def plan_interpreter(
    config_paths: Iterable[pathlib.Path],
    plan_path: pathlib.Path,
    output_path: pathlib.Path,
    plot_path: Optional[pathlib.Path],
) -> None:
    """
    The main function that interprets an entire plan file.

    However, if you are receiving one action at a time from an executor, you may prefer to import
    this module and call yaml_action_from_pddl() directly.
    """
    config = {}
    for config_path in config_paths:
        config.update(load_yaml(config_path))
    pddl_actions = parse_plan(plan_path)
    yaml_actions = filter_none(
        [
            yaml_plan_action_from_pddl(plan_action, config)
            for plan_action in pddl_actions
        ]
    )

    save_plan_yaml(output_path, yaml_actions)

    if plot_path is not None:
        plot_plan(plot_path, yaml_actions)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    "Custom formatter for argparse that combines mixins."


def main():
    "Parse arguments and invoke plan_interpreter()."
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "--config",
        help="Comma-separated list of paths to YAML problem config inputs (only static config needed)",
        type=path_list,
        default=DEFAULT_CONFIGS,
    )
    parser.add_argument(
        "--plan",
        help="Path to input plan generated by PDDL planner (parser currently tuned for POPF idiosyncrasies)",
        type=pathlib.Path,
        default=DATA_DIR / "sample_output_plan.txt",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Path for output converted to YAML format",
        type=pathlib.Path,
        default=DATA_DIR / "sample_output_plan.yaml",
    )
    parser.add_argument(
        "--plot",
        help="Write plot to specified path",
        type=lambda arg: None if arg is None else pathlib.Path(arg),
        default=None,
    )
    args = parser.parse_args()

    plan_interpreter(
        config_paths=args.config,
        plan_path=args.plan,
        output_path=args.output,
        plot_path=args.plot,
    )
    return 0

if __name__ == "__main__":
    sys.exit(main())
