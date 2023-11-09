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
Generates a PDDL problem specification for the survey domain in domain_survey.pddl.  Takes as input
a PDDL problem template and higher-level static and dynamic configuration specified in YAML.

The generator takes care of error-prone repetitive tasks like asserting predicates for which
locations are different, which robots are different, asserting a need-stereo predicate for every
completed-stereo goal, making the initial location-available predicates agree with the initial robot
locations, etc.
"""

import argparse
import itertools
import pathlib
import re
import yaml
from typing import Any, Dict, Generator, Iterable, Sequence, T, Tuple


GOAL_TYPE_OPTIONS = ("panorama", "stereo", "robot_at")


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
        run = goal["run"]
        return f"(completed-panorama {robot} {order} {location} {run})"
    elif goal_type == "stereo":
        robot = goal["robot"]
        order = goal["order"]
        trajectory = goal["trajectory"]
        run = goal["run"]
        traj_info = config_static["stereo"][trajectory]
        base = traj_info["base_location"]
        bound = traj_info["bound_location"]
        return f"(completed-stereo {robot} {order} {base} {bound} {run})"
    elif goal_type == "robot_at":
        robot = goal["robot"]
        location = goal["location"]
        return f"(robot-at {robot} {location})"
    assert False, "Never reach this point"


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
                "Expected template param in {self.params.keys()}, got {{{{ {param} }}}}"
            )
        return self.params[param]


def comment_for_pddl(text: str) -> str:
    """
    Return the result of commenting `text` using PDDL (Lisp-like) comment syntax.
    """
    return ";; " + text.replace("\n", "\n;; ")


def problem_generator(
    problem_template_path: pathlib.Path,
    config_dynamic_path: pathlib.Path,
    config_static_path: pathlib.Path,
    output_path: pathlib.Path,
) -> None:
    """
    The main function that generates the problem.
    """
    problem_template = problem_template_path.read_text()
    config_dynamic = load_yaml(config_dynamic_path)
    config_static = load_yaml(config_static_path)
    params = {}

    params["header"] = (
        ";; Auto-generated by problem_generator.py. Do not edit!\n"
        f";; Problem template: {problem_template_path}\n"
        f";; Dynamic config: {config_dynamic_path}\n"
        f";; Static config: {config_static_path}\n"
    )

    bays = list(config_static["bays"].keys())
    bogus_bays = config_static["bogus_bays"]
    all_bays = sorted(bays + bogus_bays)
    params["bays"] = " ".join(all_bays)

    berths = config_static["berths"]
    params["berths"] = " ".join(berths)

    robots = config_static["robots"]
    params["robots"] = " ".join(robots)

    num_orders = config_static["num_orders"]
    params["orders"] = " ".join([f"o{i}" for i in range(num_orders)])

    yaml_goals = config_dynamic["goals"]
    pddl_goals = [pddl_goal_from_yaml(goal, config_static) for goal in yaml_goals]
    params["goals"] = indent_lines(pddl_goals, 12)

    move_connected_lines = [
        f"(move-connected {a} {b})" for a, b in both_ways(pairwise(all_bays))
    ]
    params["move_connected_predicates"] = indent_lines(move_connected_lines, 8)

    location_real_lines = [f"(location-real {bay})" for bay in bays]
    params["location_real_predicates"] = indent_lines(location_real_lines, 8)

    candidates = (("jem_bay7", "berth1"), ("jem_bay7", "berth2"))
    dock_connected_lines = [
        f"(dock-connected {bay} {berth})"
        for bay, berth in candidates
        if bay in bays and berth in berths
    ]
    params["dock_connected_predicates"] = indent_lines(dock_connected_lines, 8)

    robots_different_lines = [
        f"(robots-different {a} {b})" for a, b in distinct_pairs(robots)
    ]
    params["robots_different_predicates"] = indent_lines(robots_different_lines, 8)

    locations_different_lines = [
        f"(locations-different {a} {b})" for a, b in distinct_pairs(all_bays)
    ]
    params["locations_different_predicates"] = indent_lines(
        locations_different_lines, 8
    )

    robot_available_lines = [f"(robot-available {robot})" for robot in robots]
    params["robot_available_predicates"] = indent_lines(robot_available_lines, 8)

    init = config_dynamic["init"]
    robot_at_lines = [
        f"(robot-at {robot} {init[robot]['location']})" for robot in robots
    ]
    params["robot_at_predicates"] = indent_lines(robot_at_lines, 8)

    all_locations = all_bays + berths
    occupied_locations = [init[robot]["location"] for robot in robots]
    available_locations = sorted(set(all_locations).difference(occupied_locations))
    location_available_lines = [
        f"(location-available {location})" for location in available_locations
    ]
    params["location_available_predicates"] = indent_lines(location_available_lines, 8)

    need_stereo_lines = [
        goal.replace("completed-stereo", "need-stereo")
        for goal in pddl_goals
        if "completed-stereo" in goal
    ]
    params["need_stereo_predicates"] = indent_lines(need_stereo_lines, 8)

    order_identity_lines = [f"(= (order-identity o{i}) {i})" for i in range(num_orders)]
    params["order_identity_fluents"] = indent_lines(order_identity_lines, 8)

    robot_order_lines = [f"(= (robot-order {robot}) -1)" for robot in robots]
    params["robot_order_fluents"] = indent_lines(robot_order_lines, 8)

    params["config_dynamic"] = comment_for_pddl(config_dynamic_path.read_text())
    params["config_static"] = comment_for_pddl(config_static_path.read_text())

    # print(yaml.safe_dump(params, indent=4, sort_keys=False))
    filled_template = TEMPLATE_SUBST_REGEX.sub(TemplateFiller(params), problem_template)
    output_path.write_text(filled_template)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    pass


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "--problem-template",
        help="Path to input PDDL problem template",
        type=pathlib.Path,
        default="jem_survey_template.pddl",
    )
    parser.add_argument(
        "--config-dynamic",
        help="Path to input dynamic problem config YAML (goals, initial state)",
        type=pathlib.Path,
        default="jem_survey_dynamic.yaml",
    )
    parser.add_argument(
        "--config-static",
        help="Path to input static problem config YAML (module geometry, available stereo surveys, etc.)",
        type=pathlib.Path,
        default="jem_survey_static.yaml",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Path for output PDDL problem",
        type=pathlib.Path,
        default="problem_jem_survey.pddl",
    )
    args = parser.parse_args()

    problem_generator(
        problem_template_path=args.problem_template,
        config_dynamic_path=args.config_dynamic,
        config_static_path=args.config_static,
        output_path=args.output,
    )


if __name__ == "__main__":
    main()
