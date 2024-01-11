#!/usr/bin/env python3

"""
Custom planner for JEM survey domain.
"""

import argparse
import heapq
import io
import itertools
import pathlib
import sys
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Type, TypeVar

import pyparsing as pp
import yaml

from problem_generator import PDDL_DIR

LocationName = str  # Names PDDL object of type location
LocationIndex = int  # Index of location in CONFIG.locations
RobotName = str  # Names PDDL object of type robot
Duration = float  # Time duration in seconds
Cost = int  # Count of moves
CostMap = List[Cost]  # Interpreted as mapping of LocationIndex -> Cost
RobotStatus = str  # Options: ACTIVE, BLOCKED, DONE
Timestamp = float  # Elapsed time since start of sim in seconds
EventCallback = Callable[["SimState"], None]
ExecutionTrace = List[Tuple[Timestamp, "Action", Duration]]
OrderName = str  # Names PDDL object of type order
PddlExpression = Any  # Actually a pp.ParseResults. But 'Any' satisfies mypy.
PddlActionName = str  # Names PDDL action
T = TypeVar("T")  # So we can assign parametric types below
PddlTypeName = str  # Names PDDL object type
PddlObjectName = str  # Names PDDL object

UNREACHABLE_COST = 999


class Config:
    "Container for custom planner config."

    def __init__(
        self,
        robots: List[RobotName],
        locations: List[LocationName],
        neighbors: List[List[LocationIndex]],
        action_durations: Dict[PddlActionName, Duration],
    ):
        """
        :param robots: PDDL objects of type robot
        :param locations: PDDL objects of type location
        :param neighbors: Maps each LocationIndex to a list of neighbor indices.
        :param action_durations: Maps each PDDL action to its duration in seconds.
        """
        self.robots = robots
        self.locations = locations
        self.neighbors = neighbors
        self.action_durations = action_durations
        self.location_lookup = {name: ind for ind, name in enumerate(locations)}


# Will override these empty placeholder values during PDDL parsing phase
CONFIG = Config(robots=[], locations=[], neighbors=[], action_durations={})


def get_cost_to_goal(goal_region: Iterable[LocationIndex]) -> CostMap:
    """
    Return a cost array that maps each location index to its cost to the nearest goal location in
    `goal_region` (measured in number of moves).
    """
    result = [UNREACHABLE_COST] * len(CONFIG.locations)
    opens = [(g, 0) for g in goal_region]
    while opens:
        curr, curr_cost = opens.pop(0)
        if curr_cost < result[curr]:
            result[curr] = curr_cost
            next_cost = curr_cost + 1
            for n in CONFIG.neighbors[curr]:
                opens.append((n, next_cost))
    return result


def get_best_neighbor(cost_map: CostMap, current_pos: LocationIndex) -> LocationIndex:
    """
    Return the neighbor of `current_pos` that has lowest cost in `cost_map` (the logical target
    location of the next move). Raise RuntimeError if the goal is unreachable  from `current_pos`.
    """
    if cost_map[current_pos] == UNREACHABLE_COST:
        raise RuntimeError(
            f"Can't reach goal region from {CONFIG.locations[current_pos]}"
        )
    return min(CONFIG.neighbors[current_pos], key=cost_map.__getitem__)


def get_move_location(
    goal_region: Iterable[LocationIndex], current_pos: LocationIndex
) -> LocationIndex:
    """
    Return the neighbor of `current_pos` that is closest to `goal_region` (the logical target
    location of the next move). Raise RuntimeError if `goal_region` is unreachable  from
    `current_pos`.
    """
    return get_best_neighbor(get_cost_to_goal(goal_region), current_pos)


def is_location_berth(location: LocationName) -> bool:
    "Return True if the location is a berth."
    return "berth" in location


def is_location_flying(location: LocationName) -> bool:
    "Return True if the location is a flying location."
    return "bay" in location


class Action(ABC):
    "Abstract class representing an action that can be invoked by the planner."

    def __init__(self, robot: RobotName):
        """
        :param robot: The robot that is executing the action.
        """
        self.robot = robot

    def get_duration(self) -> Duration:
        "Return the estimated duration of the action in seconds."
        return CONFIG.action_durations[self.get_pddl_name()]

    def invalid_reason(  # pylint: disable=unused-argument,no-self-use
        self, sim_state: "SimState"
    ) -> str:
        """
        If action is invalid in `sim_state`, return the reason why. If valid, return an empty
        string. The default implementation always returns an empty string. Derived classes can
        override as needed.
        """
        return ""

    def is_valid(self, sim_state: "SimState") -> bool:
        """
        Return True if the action can be executed in `sim_state`.
        """
        return self.invalid_reason(sim_state) == ""

    def get_pddl_name(self) -> str:
        "Return the name of this action type in the PDDL model."
        return self.__class__.__name__.replace("Action", "").lower()

    @abstractmethod
    def get_pddl_args(self) -> List[Any]:
        "Return the arguments of this action type in the PDDL model."

    def __repr__(self):
        args_str = " ".join((str(arg) for arg in self.get_pddl_args()))
        return f"({self.get_pddl_name()} {args_str})"

    def start(self, sim_state: "SimState") -> None:
        """
        Modify `sim_state` as needed at the beginning of action execution. Derived classes that
        need to extend start() should typically invoke the parent method.
        """
        sim_state.trace.append((sim_state.elapsed_time, self, self.get_duration()))
        robot_state = sim_state.robot_states[self.robot]
        robot_state.action = self
        # It seems to be a PDDL convention to separate events by an epsilon time difference
        # to prevent ambiguity about ordering.
        sim_state.elapsed_time += 0.001

    def end(self, sim_state: "SimState") -> None:
        """
        Modify `sim_state` as needed at the end of action execution. Derived classes that need to
        extend end() should typically invoke the parent method.
        """
        robot_state = sim_state.robot_states[self.robot]
        robot_state.action = None
        # It seems to be a PDDL convention to separate events by an epsilon time difference
        # to prevent ambiguity about ordering.
        sim_state.elapsed_time += 0.001

    def apply(self, sim_state: "SimState") -> None:
        """
        Apply the action, modifying `sim_state`.
        """
        end_time = sim_state.elapsed_time + self.get_duration()
        self.start(sim_state)
        heapq.heappush(sim_state.events, (end_time, self.end))


class RobotState:
    "Class representing the state of a robot."

    def __init__(
        self, pos: LocationName, action: Optional[Action], reserved: List[LocationName]
    ):
        """
        :param pos: The current location of the robot.
        :param action: The action the robot is currently executing (or None if it is idle).
        :param reserved: A list of locations the robot currently has reserved. (Places the robot
            is expected to move through given the action it is currently executing.)
        """
        self.pos = pos
        self.action = action
        self.reserved = reserved

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {"pos": self.pos, "action": repr(self.action), "reserved": self.reserved}


class SimState:
    "Class representing the overall state of the multi-robot sim."

    def __init__(self, robot_states: Dict[RobotName, RobotState]):
        """
        :param robot_states: Initial robot states.
        """
        self.elapsed_time = 0.0
        self.robot_states = robot_states
        self.events: List[Tuple[Timestamp, EventCallback]] = []
        self.trace: ExecutionTrace = []
        self.completed: Dict[Any, bool] = {}

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {
            "elapsed_time": self.elapsed_time,
            "robot_states": {
                robot: state.get_dump_dict()
                for robot, state in self.robot_states.items()
            },
            "events": sorted(self.events),
            "trace": [
                {"timestamp": t, "action": repr(a), "duration": d}
                for t, a, d in self.trace
            ],
            "completed": self.completed,
        }

    def warp(self) -> None:
        """
        Warp the simulation forward in time to the next queued event and apply its event callback.
        AKA "wait for something to happen".
        """
        if not self.events:
            return
        event_time, event_func = heapq.heappop(self.events)
        self.elapsed_time = event_time
        event_func(self)


class MarkCompleteAction(
    Action
):  # pylint: disable=abstract-method # ok in abstract class
    """
    Class representing an action that explicitly marks itself complete when it finishes executing.
    """

    def end(self, sim_state: SimState) -> None:
        super().end(sim_state)
        sim_state.completed[repr(self)] = True

    def is_complete(self, sim_state: SimState) -> bool:
        """
        Return True if this action has been completed in `sim_state`.
        """
        return sim_state.completed.get(repr(self), False)


def get_collision_check_locations(
    from_pos: LocationName, to_pos: LocationName
) -> List[str]:
    """
    Return neighbors of `to_pos` that are distinct from `from_pos` and are flying locations. A move
    is invalid if one of these collision check locations is reserved by another robot. The PDDL
    models of some actions also require collision check locations to be specified as arguments.
    """
    to_ind = CONFIG.location_lookup[to_pos]
    to_neighbors_ind = CONFIG.neighbors[to_ind]
    to_neighbors = [CONFIG.locations[n] for n in to_neighbors_ind]
    return [n for n in to_neighbors if is_location_flying(n) and n != from_pos]


def get_collision_reason(
    from_pos: LocationName, to_pos: LocationName, robot: RobotName, sim_state: SimState
) -> str:
    """
    Return the reason why a move from `from_pos` to `to_pos` with `robot` in `sim_state` fails
    collision checking. Return an empty string if the move passes the collision check.
    """
    check_locs = get_collision_check_locations(from_pos, to_pos)
    other_robots = [r for r in CONFIG.robots if r != robot]
    others_reserved = set(
        itertools.chain(
            *(sim_state.robot_states[robot].reserved for robot in other_robots)
        )
    )
    reserved_check_locs = others_reserved.intersection(check_locs)
    if reserved_check_locs:
        return f"Expected no collision check locations adjacent to `to_pos` {to_pos} to be reserved by other robots, but these are: {reserved_check_locs}"
    return ""  # ok


class AbstractMoveAction(Action):
    "Class representing a single-step move action."

    def __init__(self, robot: RobotName, from_pos: LocationName, to: LocationName):
        """
        :param robot: The robot executing the action.
        :param from_pos: The robot's starting location (only for compatibility with PDDL action).
        :param to: The location the robot should move to.
        """
        super().__init__(robot)
        self.from_pos = from_pos
        self.to = to

    def get_collision_check_locations(self) -> List[str]:
        """
        Return collision check locations for this move action.
        """
        return get_collision_check_locations(self.from_pos, self.to)

    def get_pddl_args(self) -> List[Any]:
        return [self.robot, self.from_pos, self.to]

    def invalid_reason(self, sim_state: SimState) -> str:
        robot_state = sim_state.robot_states[self.robot]
        from_ind = CONFIG.location_lookup[robot_state.pos]
        to_ind = CONFIG.location_lookup[self.to]

        # The robot can only move to an immediate neighbor of its current location
        if to_ind not in CONFIG.neighbors[from_ind]:
            return f"Expected `to` {self.to} to be an immediate neighbor of `from_pos` {self.from_pos}"

        # The robot can't move to a location reserved by another robot.
        for robot, robot_state in sim_state.robot_states.items():
            if robot != self.robot:
                if self.to in robot_state.reserved:
                    return f"Found `to` {self.to} is reserved by other robot {robot}"

        # Collision check locations must not be reserved.
        collision_reason = get_collision_reason(
            self.from_pos, self.to, self.robot, sim_state
        )
        if collision_reason:
            return collision_reason

        return ""  # valid

    def start(self, sim_state: SimState) -> None:
        super().start(sim_state)
        robot_state = sim_state.robot_states[self.robot]
        robot_state.reserved = [robot_state.pos, self.to]

    def end(self, sim_state: SimState) -> None:
        super().end(sim_state)
        robot_state = sim_state.robot_states[self.robot]
        robot_state.reserved = [self.to]
        robot_state.pos = self.to


class MoveAction(AbstractMoveAction):
    "Class representing a move action from one flying location to another."

    def get_pddl_args(self) -> List[Any]:
        # One check location argument is required for move
        return super().get_pddl_args() + self.get_collision_check_locations()[:1]

    def invalid_reason(self, sim_state: SimState) -> str:
        super_reason = super().invalid_reason(sim_state)
        if super_reason:
            return super_reason
        if not is_location_flying(self.from_pos):
            return f"Expected `from_pos` {self.from_pos} to be a flying location"
        if not is_location_flying(self.to):
            return f"Expected `to` {self.to} to be a flying location"
        return ""  # ok


class DockAction(AbstractMoveAction):
    "Class representing a dock action (from a flying location to a dock berth)."

    def invalid_reason(self, sim_state: SimState) -> str:
        super_reason = super().invalid_reason(sim_state)
        if super_reason:
            return super_reason
        if not is_location_flying(self.from_pos):
            return f"Expected `from_pos` {self.from_pos} to be a flying location"
        if not is_location_berth(self.to):
            return f"Expected `to` {self.to} to be a berth"
        return ""  # ok


class UndockAction(AbstractMoveAction):
    "Class representing an undock action (from a dock berth to a flying location)."

    def get_pddl_args(self) -> List[Any]:
        # Two check location arguments are required for undock
        return super().get_pddl_args() + self.get_collision_check_locations()[:2]

    def invalid_reason(self, sim_state: SimState) -> str:
        super_reason = super().invalid_reason(sim_state)
        if super_reason:
            return super_reason
        if not is_location_berth(self.from_pos):
            return f"Expected `from_pos` {self.from_pos} to be a berth"
        if not is_location_flying(self.to):
            return f"Expected `to` {self.to} to be a flying location"
        return ""  # ok


class PanoramaAction(MarkCompleteAction):
    "Class representing a panorama action."

    def __init__(self, robot: RobotName, order: OrderName, location: LocationName):
        super().__init__(robot)
        self.order = order
        self.location = location

    def get_pddl_args(self) -> List[Any]:
        return [self.robot, self.order, self.location]

    def invalid_reason(self, sim_state: SimState) -> str:
        robot_state = sim_state.robot_states[self.robot]
        if robot_state.pos != self.location:
            return f"Expected robot position {robot_state.pos} to match desired panorama location {self.location}"
        return ""  # ok


class StereoAction(MarkCompleteAction):
    "Class representing a stereo survey action."

    def __init__(
        self,
        robot: RobotName,
        order: OrderName,
        base: LocationName,
        bound: LocationName,
    ):
        super().__init__(robot)
        self.order = order
        self.base = base
        self.bound = bound

    def get_pddl_args(self) -> List[Any]:
        # Two collision check arguments are required for stereo action
        check_locs = get_collision_check_locations(self.base, self.bound)[:2]
        return [self.robot, self.order, self.base, self.bound] + check_locs

    def invalid_reason(self, sim_state: SimState) -> str:
        robot_state = sim_state.robot_states[self.robot]
        if robot_state.pos != self.base:
            return f"Expected robot position {robot_state.pos} to match stereo survey base {self.base}"

        # Collision check locations must not be reserved
        collision_reason = get_collision_reason(
            self.base, self.bound, self.robot, sim_state
        )
        if collision_reason:
            return collision_reason

        return ""  # ok

    def start(self, sim_state: SimState) -> None:
        super().start(sim_state)
        robot_state = sim_state.robot_states[self.robot]
        robot_state.reserved = [self.base, self.bound]

    def end(self, sim_state: SimState) -> None:
        super().end(sim_state)
        robot_state = sim_state.robot_states[self.robot]
        robot_state.reserved = [self.base]


class Goal(ABC):
    "Class representing an abstract goal."

    def __init__(self, robot: RobotName):
        """
        :param robot: The robot primarily responsible for achieving the goal.
        """
        self.robot = robot

    @abstractmethod
    def is_complete(self, exec_state: "ExecState") -> bool:
        "Return True if the goal has been completed in `exec_state`."

    @abstractmethod
    def get_next_action(self, exec_state: "ExecState") -> Action:
        "Return the next action to apply to achieve the goal given `exec_state`."


class RobotExecState:
    "Class representing the execution state of a robot."

    def __init__(self, goals):
        """
        :param goals: The sequence of goals that the robot should achieve.
        """
        self.goals = goals
        self.goal_index = 0
        self.goal_start_time: Optional[Timestamp] = None
        self.status = "INIT"
        self.blocked_action: Optional[Action] = None
        self.blocked_reason = ""

    def get_goal(self) -> Optional[Goal]:
        """
        Return the currently active goal, or None if all goals have been achieved.
        """
        if self.goal_index >= len(self.goals):
            return None
        return self.goals[self.goal_index]

    def is_active(self) -> bool:
        "Return True if the robot is actively working on a goal."
        return self.status == "ACTIVE"

    def is_done(self) -> bool:
        "Return True if the robot has achieved all of its goals."
        return self.status == "DONE"

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        result = {
            "goals": [repr(g) for g in self.goals],
            "goal_index": self.goal_index,
            "goal_start_time": self.goal_start_time,
            "status": self.status,
        }
        if self.status == "BLOCKED":
            result["blocked_action"] = repr(self.blocked_action)
            result["blocked_reason"] = repr(self.blocked_reason)
        return result


class ExecState:
    "Class representing the execution state of the multi-robot system."

    def __init__(
        self, sim_state: SimState, robot_exec_states: Dict[RobotName, RobotExecState]
    ):
        """
        :param sim_state: The initial simulation state.
        :param robot_states: The initial execution states of the robots in the multi-robot system.
        """
        self.sim_state = sim_state
        self.robot_exec_states = robot_exec_states

    def is_any_robot_active(self):
        "Return True if any robot is actively working on a goal."
        return any((rstate.is_active() for rstate in self.robot_exec_states.values()))

    def are_all_robots_done(self):
        "Return True if all robots have achieved all of their goals."
        return all((rstate.is_done() for rstate in self.robot_exec_states.values()))

    def call_next_action_internal(self, robot: RobotName) -> RobotStatus:
        "Helper for call_next_action() that returns the updated robot status."
        robot_exec_state = self.robot_exec_states[robot]
        while True:
            robot_goal = robot_exec_state.get_goal()
            if robot_goal is None:
                return "DONE"
            if robot_goal.is_complete(self):
                robot_exec_state.goal_index += 1
                robot_exec_state.goal_start_time = self.sim_state.elapsed_time
                continue
            break
        action = robot_goal.get_next_action(self)

        invalid_reason = action.invalid_reason(self.sim_state)
        if invalid_reason:
            robot_exec_state.blocked_action = action
            robot_exec_state.blocked_reason = invalid_reason
            return "BLOCKED"
        robot_exec_state.blocked_action = None
        robot_exec_state.blocked_reason = ""

        action.apply(self.sim_state)
        return "ACTIVE"

    def call_next_action(self, robot: RobotName) -> None:
        """
        Iterate through goals, starting with the current goal, until reaching a goal that is not
        completed yet, and apply that goal's next action if it is valid in the current state.
        Update the robot status: DONE = completed all goals, BLOCKED = the next action
        to perform is not (yet) valid, ACTIVE = robot is actively working on a goal.
        """
        robot_exec_state = self.robot_exec_states[robot]
        robot_exec_state.status = self.call_next_action_internal(robot)

    def run_step(self) -> None:
        """
        Try to apply each robot's next action if it is idle.
        """
        for robot in CONFIG.robots:
            robot_state = self.sim_state.robot_states[robot]
            if robot_state.action is None:
                self.call_next_action(robot)

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {
            "sim_state": self.sim_state.get_dump_dict(),
            "robot_exec_states": {
                robot: state.get_dump_dict()
                for robot, state in self.robot_exec_states.items()
            },
        }

    def __repr__(self) -> str:
        return yaml.safe_dump(self.get_dump_dict(), sort_keys=False)

    def run(self) -> None:
        """
        Achieve all goals in the multi-robot system.
        """
        while True:
            self.run_step()
            self.sim_state.warp()
            if self.are_all_robots_done():
                break
            if not self.is_any_robot_active():
                # print_trace(self.sim_state.trace)
                print(self)
                raise RuntimeError(
                    "Can't achieve all goals. Not done but no active robots!"
                )


class MoveGoal(Goal):
    "Class representing a move goal (may require multiple single-step move actions)."

    def __init__(self, robot: RobotName, to: LocationName):
        """
        :param robot: The robot executing the action.
        :param to: The location the robot should move to.
        """
        super().__init__(robot)
        self.to = to

    def __repr__(self):
        return f"(robot-at {self.robot} {self.to})"

    def is_complete(self, exec_state: ExecState) -> bool:
        "Return True if the robot is already at the desired location in `exec_state`."
        robot_state = exec_state.sim_state.robot_states[self.robot]
        return robot_state.pos == self.to

    def get_next_action(self, exec_state: ExecState) -> Action:
        "Return a single-step move action toward the desired location from `exec_state`."
        robot_state = exec_state.sim_state.robot_states[self.robot]
        to_ind = CONFIG.location_lookup[self.to]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        next_ind = get_move_location(goal_region=[to_ind], current_pos=pos_ind)
        next_loc = CONFIG.locations[next_ind]
        if is_location_berth(next_loc):
            action_type: Type[AbstractMoveAction] = DockAction
        elif is_location_berth(robot_state.pos):
            action_type = UndockAction
        else:
            action_type = MoveAction
        return action_type(robot=self.robot, from_pos=robot_state.pos, to=next_loc)


class MarkCompleteGoal(Goal):
    "Class representing a goal that is complete when a corresponding MarkCompleteAction finishes."

    @abstractmethod
    def get_completing_action(self) -> MarkCompleteAction:
        "Return the MarkCompleteAction whose completion satisfies this MarkCompleteGoal."

    def __repr__(self) -> str:
        return repr(self.get_completing_action()).replace("(", "(completed-", 1)

    def is_complete(self, exec_state: ExecState) -> bool:
        return self.get_completing_action().is_complete(exec_state.sim_state)


class PanoramaGoal(MarkCompleteGoal):
    "Class representing a panorama goal."

    def __init__(self, robot: RobotName, order: OrderName, location: LocationName):
        """
        :param order: Ordering constraint used by generic PDDL planners, ignored by this planner
        :param location: The location where the panorama should be collected.
        """
        super().__init__(robot)
        self.order = order
        self.location = location

    def get_completing_action(self) -> MarkCompleteAction:
        return PanoramaAction(
            robot=self.robot, order=self.order, location=self.location
        )

    def get_next_action(self, exec_state: ExecState) -> Action:
        """
        Return the next action needed to complete the panorama. The result will be either a move
        toward the desired location, or the corresponding PanoramaAction if already there.
        """
        move_goal = MoveGoal(self.robot, to=self.location)
        if not move_goal.is_complete(exec_state):
            return move_goal.get_next_action(exec_state)
        return self.get_completing_action()


class StereoGoal(MarkCompleteGoal):
    "Class representing a stereo survey goal."

    def __init__(
        self,
        robot: RobotName,
        order: OrderName,
        base: LocationName,
        bound: LocationName,
    ):
        """
        :param order: Ordering constraint used by generic PDDL planners, ignored by this planner
        :param base: The location where the stereo survey starts and ends.
        :param bound: The other end of the interval of locations that the robot visits during the survey.
        """
        super().__init__(robot)
        self.order = order
        self.base = base
        self.bound = bound

    def get_completing_action(self) -> MarkCompleteAction:
        return StereoAction(
            robot=self.robot, order=self.order, base=self.base, bound=self.bound
        )

    def get_next_action(self, exec_state: ExecState) -> Action:
        """
        Return the next action needed to complete the survey. The result will be either a move
        toward the desired location, or the corresponding StereoAction if already there.
        """
        move_goal = MoveGoal(self.robot, to=self.base)
        if not move_goal.is_complete(exec_state):
            return move_goal.get_next_action(exec_state)
        return self.get_completing_action()


def get_trace(trace: ExecutionTrace) -> str:
    "Return `trace` formatted in the standard PDDL plan output format used by POPF."
    out = io.StringIO()
    for timestamp, action, duration in trace:
        print(f"{timestamp:.3f}: {action} [{duration:.3f}]", file=out)
    return out.getvalue()


def parse_pddl(input_path: pathlib.Path) -> PddlExpression:
    """
    Return the result of parsing the file at `input_path` as a LISP S-expression (encompasses
    PDDL domains and problem instances).
    """
    comment = pp.Regex(r";.*").setName("LISP style comment")
    parser = pp.nestedExpr().ignore(comment)
    return parser.parseString(input_path.read_text())[0]


def get_action_durations(domain: PddlExpression) -> Dict[PddlActionName, Duration]:
    """
    Return action durations parsed from `domain`.
    """
    actions = [e for e in domain[2:] if e[0] == ":durative-action"]
    action_durations = {}
    for action in actions:
        action_name = action[1]
        duration_index = action.asList().index(":duration")
        duration_arg = action[duration_index + 1]
        op, variable, duration_str = duration_arg
        assert (
            op == "=" and variable == "?duration"
        ), f"Expected :duration arg to have the form (= ?duration ...), got {duration_arg}"
        try:
            duration = float(duration_str)
        except ValueError:
            raise RuntimeError(
                f"Expected duration value to be a float, got {repr(duration_str)}"
            )
        action_durations[action_name] = duration
    return action_durations


def get_location_config(problem: PddlExpression) -> Dict[str, Any]:
    """
    Return location configuration (available locations and neighbor lists) parsed from `problem`.
    """
    (init_predicates,) = [e for e in problem[2:] if e[0] == ":init"]
    move_connected_predicates = [p for p in init_predicates if p[0] == "move-connected"]
    dock_connected_predicates = [p for p in init_predicates if p[0] == "dock-connected"]
    neighbor_edges = [(loc1, loc2) for pred, loc1, loc2 in move_connected_predicates]

    # add dock-connected edges in both directions
    neighbor_edges += [(loc1, loc2) for pred, loc1, loc2 in dock_connected_predicates]
    neighbor_edges += [(loc2, loc1) for pred, loc1, loc2 in dock_connected_predicates]

    locations = sorted(set(itertools.chain(*neighbor_edges)))
    location_lookup = {name: ind for ind, name in enumerate(locations)}
    neighbor_edge_ind = [
        (location_lookup[loc1], location_lookup[loc2]) for loc1, loc2 in neighbor_edges
    ]
    neighbors = [
        [loc2 for loc1, loc2 in neighbor_edge_ind if loc1 == loc_ind]
        for loc_ind in range(len(locations))
    ]
    return {
        "locations": locations,
        "location_lookup": location_lookup,
        "neighbors": neighbors,
    }


def get_goal_from_pddl(goal_expr: PddlExpression) -> Optional[Goal]:
    """
    Return the custom planner goal type corresponding to PDDL `goal_expr`.
    """
    goal_type = goal_expr[0]
    if goal_type == "completed-panorama":
        return PanoramaGoal(
            robot=goal_expr[1], order=goal_expr[2], location=goal_expr[3]
        )
    if goal_type == "completed-stereo":
        return StereoGoal(
            robot=goal_expr[1],
            order=goal_expr[2],
            base=goal_expr[3],
            bound=goal_expr[4],
        )
    if goal_type == "robot-at":
        return MoveGoal(robot=goal_expr[1], to=goal_expr[2])

    print(
        f"WARNING: get_goal_from_pddl(): Can't map goal_type {goal_type} to a custom planner goal type, ignoring",
        file=sys.stderr,
    )
    return None


def filter_none(seq: Iterable[Optional[T]]) -> List[T]:
    "Return `seq` with None elements filtered out."
    return [elt for elt in seq if elt is not None]


def get_objects_by_type(
    problem: PddlExpression
) -> Dict[PddlTypeName, List[PddlObjectName]]:
    "Return robot names parsed from PDDL problem instance."
    (objects_clause,) = [e for e in problem[2:] if e[0] == ":objects"]
    # object_decls example: ["foo", "bar", "-", "int", "baz", "-", "float"]
    object_decls = list(objects_clause[1:])

    # desired objects_of_type in example: {"int": ["foo", "bar"], "float": ["baz"]}
    objects_of_type = {}
    while object_decls:
        type_marker_ind = object_decls.index("-")
        type_arg = object_decls[type_marker_ind + 1]
        objects_of_type[type_arg] = object_decls[:type_marker_ind]
        object_decls = object_decls[type_marker_ind + 2 :]

    return objects_of_type


def get_robot_goals(problem: PddlExpression) -> Dict[RobotName, List[Goal]]:
    """
    Return a mapping of robot name to robot goals parsed from `problem`.
    """
    (goal_clause,) = [e for e in problem[2:] if e[0] == ":goal"]
    compound_expr = goal_clause[1]
    if compound_expr[0] == "and":
        goal_exprs = compound_expr[1:]
    else:
        goal_exprs = [compound_expr]
    goals = filter_none([get_goal_from_pddl(goal_expr) for goal_expr in goal_exprs])
    robot_goals = {
        robot: [g for g in goals if g.robot == robot] for robot in CONFIG.robots
    }
    return robot_goals


def get_robot_states(problem: PddlExpression) -> Dict[RobotName, RobotState]:
    """
    Return a mapping of robot name to robot state parsed from `problem`.
    """
    robot_states = {
        robot: RobotState(pos="", action=None, reserved=[]) for robot in CONFIG.robots
    }
    (init_clause,) = [e for e in problem[2:] if e[0] == ":init"]
    init_predicates = init_clause[1:]
    robot_at_predicates = [p for p in init_predicates if p[0] == "robot-at"]
    for _, robot, pos in robot_at_predicates:
        robot_states[robot].pos = pos
        robot_states[robot].reserved = [pos]
    return robot_states


def survey_planner(domain_path: pathlib.Path, problem_path: pathlib.Path):
    "Primary driver function for custom planning."

    domain_expr = parse_pddl(domain_path)
    CONFIG.action_durations = get_action_durations(domain_expr)

    problem_expr = parse_pddl(problem_path)
    CONFIG.robots = get_objects_by_type(problem_expr)["robot"]

    location_config = get_location_config(problem_expr)
    CONFIG.locations = location_config["locations"]
    CONFIG.location_lookup = location_config["location_lookup"]
    CONFIG.neighbors = location_config["neighbors"]

    robot_goals = get_robot_goals(problem_expr)
    robot_states = get_robot_states(problem_expr)

    sim_state = SimState(robot_states=robot_states)
    robot_exec_states = {
        robot: RobotExecState(robot_goals[robot]) for robot in CONFIG.robots
    }
    exec_state = ExecState(sim_state=sim_state, robot_exec_states=robot_exec_states)

    exec_state.run()
    print(get_trace(exec_state.sim_state.trace))


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    "Custom formatter for argparse that combines mixins."


def main():
    "Parse command-line arguments and invoke survey_planner()."
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=CustomFormatter
    )
    parser.add_argument(
        "domain",
        help="Path for input PDDL domain",
        type=pathlib.Path,
        default=PDDL_DIR / "domain_survey.pddl",
        nargs="?",
    )
    parser.add_argument(
        "problem",
        help="Path for input PDDL problem",
        type=pathlib.Path,
        default=PDDL_DIR / "problem_jem_survey.pddl",
        nargs="?",
    )
    args = parser.parse_args()

    survey_planner(domain_path=args.domain, problem_path=args.problem)


if __name__ == "__main__":
    main()
