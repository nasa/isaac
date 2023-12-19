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
import io
import itertools
import pathlib
import re
import shlex
import sys
from typing import Any, Dict, Iterable, List, Sequence, T, Tuple

import yaml

GOAL_TYPE_OPTIONS = ("panorama", "stereo", "robot_at")

THIS_DIR = pathlib.Path(__file__).resolve().parent
CWD = pathlib.Path.cwd()
DATA_DIR = (THIS_DIR / ".." / "data").resolve().relative_to(CWD)
PDDL_DIR = (THIS_DIR / ".." / "pddl").resolve().relative_to(CWD)
DEFAULT_CONFIGS = [
    DATA_DIR / "jem_survey_static.yaml",
    DATA_DIR / "jem_survey_dynamic.yaml",
]

# Type alias
YamlMapping = Dict[str, Any]

# Replace the text "{{ foo }}" in the template with the value of the foo parameter
TEMPLATE_SUBST_REGEX = re.compile(r"{{\s*([\w]+)\s*}}")


def load_yaml(yaml_path: pathlib.Path) -> YamlMapping:
    """
    Return the YAML parse result for the file at `yaml_path`.
    """
    with yaml_path.open(encoding="utf-8") as yaml_stream:
        return yaml.safe_load(yaml_stream)


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
    a, b = itertools.tee(elts)
    next(b, None)
    return zip(a, b)


def both_ways(elts: Iterable[Tuple[T, T]]) -> Iterable[Tuple[T, T]]:
    """
    Given `elts` an iterable of 2-tuples, return a new iterable that includes each 2-tuple twice,
    once in its original order and once reversed.
    """
    for a, b in elts:
        yield a, b
        yield b, a


def distinct_pairs(elts: Iterable[T]) -> Iterable[Tuple[T, T]]:
    """
    Return distinct pairs of items drawn from `elts`.
    """
    for a, b in itertools.product(elts, repeat=2):
        if a != b:
            yield a, b


class TemplateFiller:
    """
    Class for substituting parameters into a template.
    """

    def __init__(self, params):
        self.params = params

    def __call__(self, match: re.match) -> str:
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


class TerminalWriter:
    def __init__(self):
        self._buf = io.StringIO()

    def getvalue(self):
        return self._buf.getvalue()

    def command(self, cmd: str) -> None:
        self._buf.write(cmd + "\n")

    def set_instance(self, instance_name: str, pddl_type: str) -> None:
        self.command(f"set instance {instance_name} {pddl_type}")

    def set_instances(self, instance_names: Iterable[str], pddl_type: str) -> None:
        for instance_name in instance_names:
            self.set_instance(instance_name, pddl_type)

    def set_predicate(self, predicate: str) -> None:
        self.command(f"set predicate {predicate}")

    def set_predicates(self, predicates: Iterable[str]) -> None:
        for predicate in predicates:
            self.set_predicate(predicate)

    def set_goals(self, goals: Iterable[str]) -> None:
        self.command(f"set goal (and {' '.join(goals)})")

    def set_fluent(self, fluent: str) -> None:
        self.command(f"set function {fluent}")

    def set_fluents(self, fluents: Iterable[str]) -> None:
        for fluent in fluents:
            self.set_fluent(fluent)


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
    problem_template = problem_template_path.read_text()
    config = {}
    for config_path in config_paths:
        config.update(load_yaml(config_path))
    params = {}

    header_lines = (
        ";; Auto-generated by problem_generator.py. Do not edit!\n"
        f";; Command was: {command}\n"
        f";; Working directory was: {CWD}\n"
        f";; Problem template: {problem_template_path}\n"
    )
    for i, config_path in enumerate(config_paths):
        header_lines += f";; Config {i + 1}: {config_path}\n"
    if terminal:
        writer = TerminalWriter()
    else:
        params["header"] = header_lines

    bays = list(config["bays"].keys())
    bogus_bays = config["bogus_bays"]
    all_bays = sorted(bays + bogus_bays)
    if terminal:
        writer.set_instances(all_bays, "location")
    else:
        params["bays"] = " ".join(all_bays)

    berths = config["berths"]
    if terminal:
        writer.set_instances(berths, "location")
    else:
        params["berths"] = " ".join(berths)

    robots = config["robots"]
    if terminal:
        writer.set_instances(robots, "robot")
    else:
        params["robots"] = " ".join(robots)

    max_order = max([goal.get("order", -1) for goal in config["goals"]])
    num_orders = max_order + 1
    orders = [f"o{i}" for i in range(num_orders)]
    if terminal:
        writer.set_instances(orders, "order")
    else:
        params["orders"] = " ".join(orders)

    yaml_goals = config["goals"]
    pddl_goals = [pddl_goal_from_yaml(goal, config) for goal in yaml_goals]
    if terminal:
        writer.set_goals(pddl_goals)
    else:
        params["goals"] = indent_lines(pddl_goals, 12)

    move_connected_lines = [
        f"(move-connected {a} {b})" for a, b in both_ways(pairwise(all_bays))
    ]
    if terminal:
        writer.set_predicates(move_connected_lines)
    else:
        params["move_connected_predicates"] = indent_lines(move_connected_lines, 8)

    location_real_lines = [f"(location-real {bay})" for bay in bays]
    if terminal:
        writer.set_predicates(location_real_lines)
    else:
        params["location_real_predicates"] = indent_lines(location_real_lines, 8)

    candidates = (("jem_bay7", "berth1"), ("jem_bay7", "berth2"))
    dock_connected_lines = [
        f"(dock-connected {bay} {berth})"
        for bay, berth in candidates
        if bay in bays and berth in berths
    ]
    if terminal:
        writer.set_predicates(dock_connected_lines)
    else:
        params["dock_connected_predicates"] = indent_lines(dock_connected_lines, 8)

    robots_different_lines = [
        f"(robots-different {a} {b})" for a, b in distinct_pairs(robots)
    ]
    if terminal:
        writer.set_predicates(robots_different_lines)
    else:
        params["robots_different_predicates"] = indent_lines(robots_different_lines, 8)

    locs_different_lines = [
        f"(locations-different {a} {b})" for a, b in distinct_pairs(all_bays)
    ]
    if terminal:
        writer.set_predicates(locs_different_lines)
    else:
        params["locs_different_predicates"] = indent_lines(locs_different_lines, 8)

    robot_available_lines = [f"(robot-available {robot})" for robot in robots]
    if terminal:
        writer.set_predicates(robot_available_lines)
    else:
        params["robot_available_predicates"] = indent_lines(robot_available_lines, 8)

    init = config["init"]
    robot_at_lines = [
        f"(robot-at {robot} {init[robot]['location']})" for robot in robots
    ]
    if terminal:
        writer.set_predicates(robot_at_lines)
    else:
        params["robot_at_predicates"] = indent_lines(robot_at_lines, 8)

    all_locations = all_bays + berths
    occupied_locations = [init[robot]["location"] for robot in robots]
    available_locations = sorted(set(all_locations).difference(occupied_locations))
    location_available_lines = [
        f"(location-available {location})" for location in available_locations
    ]
    if terminal:
        writer.set_predicates(location_available_lines)
    else:
        params["location_available_predicates"] = indent_lines(location_available_lines, 8)

    need_stereo_lines = [
        goal.replace("completed-stereo", "need-stereo")
        for goal in pddl_goals
        if "completed-stereo" in goal
    ]
    if terminal:
        writer.set_predicates(need_stereo_lines)
    else:
        params["need_stereo_predicates"] = indent_lines(need_stereo_lines, 8)

    order_identity_lines = [f"(= (order-identity o{i}) {i})" for i in range(num_orders)]
    if terminal:
        writer.set_fluents(order_identity_lines)
    else:
        params["order_identity_fluents"] = indent_lines(order_identity_lines, 8)

    robot_order_lines = [f"(= (robot-order {robot}) -1)" for robot in robots]
    if terminal:
        writer.set_fluents(robot_order_lines)
    else:
        params["robot_order_fluents"] = indent_lines(robot_order_lines, 8)

    if terminal:
        output_path = pathlib.Path(str(output_path_template).replace("{ext}", ".ps2.pddl"))
        output_path.write_text(writer.getvalue())
        print(f"Wrote to {output_path}", file=sys.stderr)
    else:
        config_text = ""
        for config_path in config_paths:
            config_text += config_path.read_text()
        params["config"] = comment_for_pddl(config_text)

        filled_template = TEMPLATE_SUBST_REGEX.sub(TemplateFiller(params), problem_template)
        output_path = pathlib.Path(str(output_path_template).replace("{ext}", ".pddl"))
        output_path.write_text(filled_template)
        print(f"Wrote to {output_path}", file=sys.stderr)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def path_list(paths_text: str) -> List[pathlib.Path]:
    return [pathlib.Path(pstr) for pstr in paths_text.split(",")]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "-t", "--terminal",
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
