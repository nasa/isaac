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
Generates a problem specification for the survey domain in domain_survey.pddl. Output is either a
PDDL problem instance or the equivalent sequence of commands to pass to the PlanSys2 terminal (if
-t is specified).

Takes as input a list of higher-level configuration files specified in YAML. Normally, this is a
static config that contains non-changing information like bay coordinates, plus a dynamic config
that contains things that are likely to change during execution, like initial state and
goals. However, the split is arbitrary and you can just concatenate all the config files to get the
same effect.

The generator takes care of error-prone repetitive tasks like asserting predicates for which
locations are different, which robots are different, asserting a need-stereo predicate for every
completed-stereo goal, making the initial location-available predicates agree with the initial robot
locations, etc.
"""

import argparse
import collections
import io
import itertools
import os
import pathlib
import re
import shlex
import sys
from abc import ABC, abstractmethod
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple, TypeVar

import yaml

GOAL_TYPE_OPTIONS = ("panorama", "stereo", "robot_at", "let_other_robot_reach")

THIS_DIR = pathlib.Path(__file__).resolve().parent
CWD = pathlib.Path.cwd()
DATA_DIR = pathlib.Path(os.path.relpath(str((THIS_DIR / ".." / "data").resolve()), CWD))
PDDL_DIR = pathlib.Path(os.path.relpath(str((THIS_DIR / ".." / "pddl").resolve()), CWD))
DEFAULT_CONFIGS = [
    DATA_DIR / "survey_static.yaml",
    DATA_DIR / "jem_survey_dynamic.yaml",
]

# Type alias
YamlMapping = Dict[str, Any]
T = TypeVar("T")  # pylint: disable=invalid-name

# Replace the text "{{ foo }}" in the template with the value of the foo parameter
TEMPLATE_SUBST_REGEX = re.compile(r"{{\s*([\w]+)\s*}}")


def load_yaml(yaml_path: pathlib.Path) -> YamlMapping:
    """
    Return the YAML parse result for the file at `yaml_path`.
    """
    with yaml_path.open(encoding="utf-8") as yaml_stream:
        return yaml.safe_load(yaml_stream)


def get_stereo_traj(static_config, base, bound):
    traj_matches = [
        traj
        for traj in static_config["stereo"].values()
        if traj["base_location"] == base and traj["bound_location"] == bound
    ]
    assert (
        len(traj_matches) == 1
    ), f"Expected exactly 1 matching stereo trajectory with base {base} and bound {bound}, got {len(traj_matches)}"
    fplan = traj_matches[0]["fplan"]
    return fplan


def pddl_goal_from_yaml(goal: YamlMapping, config_static: YamlMapping) -> str:
    """
    Convert a YAML goal with named fields from the dynamic config into a PDDL goal predicate.
    """
    goal_type = goal["type"]
    assert (
        goal_type in GOAL_TYPE_OPTIONS
    ), f"Expected goal type in {GOAL_TYPE_OPTIONS}, got {goal_type}"

    if goal_type == "panorama":
        robot = goal["robot"]
        order = goal["order"]
        location = goal["location"]
        return f"(completed-panorama {robot} o{order} {location})"

    if goal_type == "stereo":
        robot = goal["robot"]
        order = goal["order"]
        trajectory = goal["trajectory"]
        traj_info = config_static["stereo"][trajectory]
        base = traj_info["base_location"]
        bound = traj_info["bound_location"]
        return f"(completed-stereo {robot} o{order} {base} {bound})"

    if goal_type == "robot_at":
        robot = goal["robot"]
        location = goal["location"]
        return f"(robot-at {robot} {location})"

    if goal_type == "let_other_robot_reach":
        robot = goal["robot"]
        location = goal["location"]
        order = goal["order"]
        return f"(completed-let-other-robot-reach {robot} o{order} {location})"

    assert False, "Never reach this point"
    return {}  # Make pylint happy


def indent_lines(lines: Sequence[str], indent: int) -> str:
    """
    Return `lines` joined with carriage returns, prepending `indent` spaces to each line after the
    first.
    """
    sep = "\n" + (" " * indent)
    return sep.join(lines)


def pairwise(elts: Iterable[T]) -> Iterable[Tuple[T, T]]:
    """
    Returns consecutive pairs drawn from `elts`. Back-port itertools.pairwise() to earlier Python
    versions.
    """
    a, b = itertools.tee(elts)  # pylint: disable=invalid-name
    next(b, None)
    return zip(a, b)


def both_ways(elts: Iterable[Tuple[T, T]]) -> Iterable[Tuple[T, T]]:
    """
    Given `elts` an iterable of 2-tuples, return a new iterable that includes each 2-tuple twice,
    once in its original order and once reversed.
    """
    for a, b in elts:  # pylint: disable=invalid-name
        yield a, b
        yield b, a


def distinct_pairs(elts: Iterable[T]) -> Iterable[Tuple[T, T]]:
    """
    Return distinct pairs of items drawn from `elts`.
    """
    # pylint: disable=invalid-name
    for a, b in itertools.product(elts, repeat=2):
        if a != b:
            yield a, b


class TemplateFiller:
    """
    Class for substituting parameters into a template.
    """

    def __init__(self, params):
        self.params = params

    def __call__(self, match: re.Match) -> str:
        param = match.group(1)
        if param not in self.params:
            raise KeyError(
                f"Template param {{{{ {param} }}}} not found in {list(self.params.keys())}"
            )
        return self.params[param]


def comment_for_pddl(text: str) -> str:
    """
    Return the result of commenting `text` using PDDL (Lisp-like) comment syntax.
    """
    return "\n".join([f";; {line}".strip() for line in text.splitlines()])


class ProblemWriter(ABC):
    "Abstract class for writing a problem intance."

    @abstractmethod
    def getvalue(self) -> str:
        "Return problem text to output."

    def set_param(self, key: str, value: str) -> None:
        "Set template parameter `key` to `value`. (This method is a no-op; override in children.)"

    @abstractmethod
    def get_extension(self) -> str:
        "Return standard extension to use for output file."

    @abstractmethod
    def declare_instance(self, instance_name: str, pddl_type: str) -> None:
        "Declare `instance_name` as an instance of `pddl_type`."

    def declare_instances(self, instance_names: Iterable[str], pddl_type: str) -> None:
        "Declare `instance_names` as instances of `pddl_type`."
        for instance_name in instance_names:
            self.declare_instance(instance_name, pddl_type)

    @abstractmethod
    def declare_predicate(self, predicate: str, section: str) -> None:
        "Declare `predicate` as a PDDL predicate in `section`."

    def declare_predicates(self, predicates: Iterable[str], section: str) -> None:
        "Declare `predicates` as PDDL predicates in `section`."
        for predicate in predicates:
            self.declare_predicate(predicate, section)

    @abstractmethod
    def declare_goals(self, goals: Iterable[str]) -> None:
        "Declare `goals` as PDDL goals. (Can only be called once, with all goals.)"

    @abstractmethod
    def declare_fluent(self, fluent: str, section: str) -> None:
        "Declare `fluent` as a PDDL fluent in `section`."

    def declare_fluents(self, fluents: Iterable[str], section: str) -> None:
        "Declare `fluents` as PDDL fluents in `section`."
        for fluent in fluents:
            self.declare_fluent(fluent, section)


class PddlWriter(ProblemWriter):
    "Class for writing a problem intance in PDDL format."

    def __init__(self, template_path: pathlib.Path):
        self.template_path = template_path
        self._params: Dict[str, str] = {}
        self._section_bufs: Mapping[str, io.StringIO] = collections.defaultdict(
            io.StringIO
        )
        self._instances_of_types: Mapping[str, List[str]] = collections.defaultdict(
            list
        )

    def get_extension(self) -> str:
        return ".pddl"

    def set_param(self, key: str, value: str) -> None:
        "Set template parameter `key` to `value`."
        self._params[key] = value

    def _get_objects_str(self) -> str:
        "Return object instance declarations in PDDL format."
        indent = " " * 8
        return "\n".join(
            (
                f"{indent}{' '.join(instances)} - {pddl_type}"
                for pddl_type, instances in self._instances_of_types.items()
            )
        ).lstrip()

    def getvalue(self) -> str:
        self._params["objects"] = self._get_objects_str()
        self._params.update(
            {
                section: buf.getvalue().strip()
                for section, buf in self._section_bufs.items()
            }
        )
        template_text = self.template_path.read_text()
        filled_template = TEMPLATE_SUBST_REGEX.sub(
            TemplateFiller(self._params), template_text
        )
        return filled_template

    def declare(self, statement: str, section: str) -> None:
        "Declare `statement` in `section`."
        indent = " " * 8
        self._section_bufs[section].write(indent + statement + "\n")

    def declare_instance(self, instance_name: str, pddl_type: str) -> None:
        self._instances_of_types[pddl_type].append(instance_name)

    def declare_predicate(self, predicate: str, section: str) -> None:
        self.declare(predicate, section)

    def declare_goals(self, goals: Iterable[str]) -> None:
        indent = " " * 12
        self.set_param("goals", "\n".join((indent + goal for goal in goals)).lstrip())

    def declare_fluent(self, fluent: str, section: str) -> None:
        self.declare(fluent, section)


class TerminalWriter(ProblemWriter):
    "Class for writing a problem instance as a sequence of PlanSys2 terminal commands."

    def __init__(self):
        self._buf = io.StringIO()

    def get_extension(self) -> str:
        return ".ps2.pddl"

    def getvalue(self) -> str:
        return self._buf.getvalue()

    def declare(self, statement: str) -> None:
        "Declare `statement`."
        self._buf.write(statement + "\n")

    def declare_instance(self, instance_name: str, pddl_type: str) -> None:
        self.declare(f"set instance {instance_name} {pddl_type}")

    def declare_predicate(self, predicate: str, section: str) -> None:
        self.declare(f"set predicate {predicate}")

    def declare_goals(self, goals: Iterable[str]) -> None:
        self.declare(f"set goal (and {' '.join(goals)})")

    def declare_fluent(self, fluent: str, section: str) -> None:
        self.declare(f"set function {fluent}")


def problem_generator(
    problem_template_path: pathlib.Path,
    config_paths: Iterable[pathlib.Path],
    output_path_template: pathlib.Path,
    command: str,
    terminal: bool,
) -> None:
    """
    The main function that generates the problem.
    """
    config = {}
    for config_path in config_paths:
        config.update(load_yaml(config_path))

    if terminal:
        writer: ProblemWriter = TerminalWriter()
    else:
        writer = PddlWriter(problem_template_path)
        header_lines = (
            ";; Auto-generated by problem_generator.py. Do not edit!\n"
            f";; Command was: {command}\n"
            f";; Working directory was: {CWD}\n"
            f";; Problem template: {problem_template_path}\n"
        )
        full_config = ""
        for i, config_path in enumerate(config_paths):
            header_lines += f";; Config {i + 1}: {config_path}\n"
            full_config += config_path.read_text()
        writer.set_param("header", header_lines)
        writer.set_param("config", comment_for_pddl(full_config))

    bays = list(config["bays"].keys())
    bogus_bays = config["bogus_bays"]
    all_bays = sorted(bays + bogus_bays)
    writer.declare_instances(all_bays, "location")

    berths = config["berths"]
    writer.declare_instances(berths, "location")

    robots = config["robots"]
    writer.declare_instances(robots, "robot")

    max_order = max([goal.get("order", -1) for goal in config["goals"]])
    num_orders = max_order + 1
    orders = [f"o{i}" for i in range(num_orders)]
    writer.declare_instances(orders, "order")

    yaml_goals = config["goals"]
    pddl_goals = [pddl_goal_from_yaml(goal, config) for goal in yaml_goals]
    writer.declare_goals(pddl_goals)

    move_connected_lines = [
        f"(move-connected {a} {b})" for a, b in both_ways(pairwise(all_bays))
    ]
    writer.declare_predicates(move_connected_lines, "static_predicates")

    location_real_lines = [f"(location-real {bay})" for bay in bays]
    writer.declare_predicates(location_real_lines, "static_predicates")

    candidates = (("jem_bay7", "berth1"), ("jem_bay7", "berth2"))
    dock_connected_lines = [
        f"(dock-connected {bay} {berth})"
        for bay, berth in candidates
        if bay in bays and berth in berths
    ]
    writer.declare_predicates(dock_connected_lines, "static_predicates")

    robots_different_lines = [
        f"(robots-different {a} {b})" for a, b in distinct_pairs(robots)
    ]
    writer.declare_predicates(robots_different_lines, "static_predicates")

    locs_different_lines = [
        f"(locations-different {a} {b})" for a, b in distinct_pairs(all_bays)
    ]
    writer.declare_predicates(locs_different_lines, "static_predicates")

    robot_available_lines = [f"(robot-available {robot})" for robot in robots]
    writer.declare_predicates(robot_available_lines, "dynamic_predicates")

    init = config["init"]
    robot_at_lines = [
        f"(robot-at {robot} {init[robot]['location']})" for robot in robots
    ]
    writer.declare_predicates(robot_at_lines, "dynamic_predicates")

    all_locations = all_bays + berths
    occupied_locations = [init[robot]["location"] for robot in robots]
    available_locations = sorted(set(all_locations).difference(occupied_locations))
    location_available_lines = [
        f"(location-available {location})" for location in available_locations
    ]
    writer.declare_predicates(location_available_lines, "dynamic_predicates")

    need_stereo_lines = [
        goal.replace("completed-stereo", "need-stereo")
        for goal in pddl_goals
        if "completed-stereo" in goal
    ]
    writer.declare_predicates(need_stereo_lines, "dynamic_predicates")

    order_identity_lines = [f"(= (order-identity o{i}) {i})" for i in range(num_orders)]
    writer.declare_fluents(order_identity_lines, "static_fluents")

    robot_order_lines = [f"(= (robot-order {robot}) -1)" for robot in robots]
    writer.declare_fluents(robot_order_lines, "dynamic_fluents")

    output_path = pathlib.Path(
        str(output_path_template).replace("{ext}", writer.get_extension())
    )
    output_path.write_text(writer.getvalue())
    print(f"Wrote to {output_path}", file=sys.stderr)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    "Custom formatter for argparse that combines mixins."


def path_list(paths_text: str) -> List[pathlib.Path]:
    "Return the list of paths parsed from comma-separated list `paths_text`."
    return [pathlib.Path(pstr) for pstr in paths_text.split(",")]


def main():
    "Parse arguments and invoke problem_generator()."
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-t",
        "--terminal",
        help="Format output for PlanSys2 terminal instead of PDDL",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--problem-template",
        help="Path to input PDDL problem template (unused if -t specified)",
        type=pathlib.Path,
        default=PDDL_DIR / "jem_survey_template.pddl",
    )
    parser.add_argument(
        "--config",
        help="Comma-separated list of paths to YAML problem config inputs",
        type=path_list,
        default=",".join([str(path) for path in DEFAULT_CONFIGS]),
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Path for output PDDL problem ({ext} filled with extension)",
        type=pathlib.Path,
        default=PDDL_DIR / "problem_jem_survey{ext}",
    )
    args = parser.parse_args()

    problem_generator(
        problem_template_path=args.problem_template,
        config_paths=args.config,
        output_path_template=args.output,
        command=shlex.join(sys.argv),
        terminal=args.terminal,
    )


if __name__ == "__main__":
    main()
