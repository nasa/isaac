#!/usr/bin/env python3

"""
Custom planner for JEM survey domain.
"""

import argparse
import enum
import heapq
import io
import itertools
import pathlib
import sys
from abc import ABC
from dataclasses import dataclass, field
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    Iterable,
    Iterator,
    List,
    NamedTuple,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
)

import pyparsing as pp
import yaml

from survey_planner.problem_generator import PDDL_DIR

LocationName = str  # Names PDDL object of type location
LocationIndex = int  # Index of location in CONFIG.locations
RobotName = str  # Names PDDL object of type robot
Duration = float  # Time duration in seconds
Cost = int  # Count of moves
CostMap = List[Cost]  # Interpreted as mapping of LocationIndex -> Cost
Timestamp = float  # Elapsed time since start of sim in seconds
EventCallback = Callable[["SimState"], None]
PendingEvent = Tuple[Timestamp, EventCallback]
ExecutionTrace = List["TraceEvent"]
OrderName = str  # Names PDDL object of type order
PddlExpression = Any  # Actually a pp.ParseResults. But 'Any' satisfies mypy.
PddlPredicate = Any  # More specific than PddlExpression, same from mypy perspective.
PddlActionName = str  # Names PDDL action
T = TypeVar("T")  # So we can assign parametric types below
PddlTypeName = str  # Names PDDL object type
PddlObjectName = str  # Names PDDL object

UNREACHABLE_COST = 999


@dataclass
class Config:
    "Container for custom planner config."

    robots: List[RobotName] = field(default_factory=list)
    "PDDL objects of type robot"

    locations: List[LocationName] = field(default_factory=list)
    "PDDL objects of type location"

    neighbors: List[List[LocationIndex]] = field(default_factory=list)
    "Maps each LocationIndex to a list of neighbor indices."

    action_durations: Dict[PddlActionName, Duration] = field(default_factory=dict)
    "Maps each PDDL action to its duration in seconds."

    def __post_init__(self) -> None:
        "Set `location_lookup` to match `locations`."

        self.location_lookup = {name: ind for ind, name in enumerate(self.locations)}
        # pylint: disable-next=pointless-string-statement  # doc string
        "Maps each PDDL LocationName to its LocationIndex."

    def update_location_lookup(self) -> None:
        "Update `location_lookup` to match `locations`."
        self.__post_init__()


# Will override fields of this empty placeholder during PDDL parsing phase
CONFIG = Config()


def get_cost_to_goal(
    goal_region: Iterable[LocationIndex],
    blocked_region: Optional[List[LocationIndex]] = None,
) -> CostMap:
    """
    Return a cost array that maps each location index to its cost to the nearest goal location in
    `goal_region` (measured in number of moves).

    :param blocked_region: If other than None, specifies locations that a robot can't move through
      on its way to `goal_region`.
    """
    result = [UNREACHABLE_COST] * len(CONFIG.locations)
    opens = [(g, 0) for g in goal_region]
    while opens:
        curr, curr_cost = opens.pop(0)
        if blocked_region is not None and curr in blocked_region:
            # blocked locations always stay at UNREACHABLE_COST
            continue
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
    goal_region: Iterable[LocationIndex],
    current_pos: LocationIndex,
    blocked_region: Optional[List[LocationIndex]] = None,
) -> LocationIndex:
    """
    Return the neighbor of `current_pos` that is closest to `goal_region` (the logical target
    location of the next move). Raise RuntimeError if `goal_region` is unreachable  from
    `current_pos`.
    """
    return get_best_neighbor(get_cost_to_goal(goal_region, blocked_region), current_pos)


def get_visited(
    from_pos: LocationIndex,
    to_region: List[LocationIndex],
    blocked_region: Optional[List[LocationIndex]] = None,
) -> List[LocationIndex]:
    """
    Return the list of locations visited while moving from `from_pos` to `to_region` along a
    shortest path, including both endpoints.
    """
    cost_map = get_cost_to_goal(to_region, blocked_region)
    assert cost_map[from_pos] != UNREACHABLE_COST
    current_pos = from_pos
    result = []
    while True:
        result.append(current_pos)
        if current_pos in to_region:
            break
        current_pos = get_best_neighbor(cost_map, current_pos)
    return result


def get_collision_radius(loc_ind: LocationIndex) -> List[LocationIndex]:
    "Return the locations that would be unsafe for robot A if robot B has `loc_ind` reserved."
    loc_name = CONFIG.locations[loc_ind]
    result = [loc_ind]
    # Include neighbors if *both* neighbors are flying locations
    if is_location_flying(loc_name):
        result += [
            n
            for n in CONFIG.neighbors[loc_ind]
            if is_location_flying(CONFIG.locations[n])
        ]
    return result


def invert_region(region: List[LocationIndex]) -> List[LocationIndex]:
    "Return locations not in `region`."
    return list(set(range(len(CONFIG.locations))).difference(region))


def get_safe_from(avoid_region: List[LocationIndex]) -> List[LocationIndex]:
    """
    Return the list of locations that would be safe for robot A if robot B is visiting
    all locations in `avoid_region`. (Flying locations need to be at least two steps apart.)
    """
    avoid_dilated: List[LocationIndex] = list(
        set().union(*(get_collision_radius(loc) for loc in avoid_region))  # type:ignore
    )
    return invert_region(avoid_dilated)


def is_location_berth(location: LocationName) -> bool:
    "Return True if the location is a berth."
    return "berth" in location


def is_location_flying(location: LocationName) -> bool:
    "Return True if the location is a flying location."
    return "bay" in location


class TraceEvent(NamedTuple):
    "Represents an event in an execution trace."
    timestamp: Timestamp
    action: "Action"
    duration: Duration

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {
            "timestamp": self.timestamp,
            "action": self.action,
            "duration": self.duration,
        }


@dataclass
class Action(ABC):
    "Represents an action that can be invoked by the planner."

    robot: RobotName
    "The robot that is executing the action."

    def get_duration(self) -> Duration:
        "Return the estimated duration of the action in seconds."
        return CONFIG.action_durations[self.get_pddl_name()]

    def invalid_reason(  # pylint: disable=unused-argument
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

    def get_pddl_args(self) -> List[Any]:
        "Return the arguments of this action type in the PDDL model."
        raise NotImplementedError()  # Note: Can't mark as @abstractmethod due to mypy limitation

    def __repr__(self):
        args_str = " ".join((str(arg) for arg in self.get_pddl_args()))
        return f"({self.get_pddl_name()} {args_str})"

    def start(self, sim_state: "SimState") -> None:
        """
        Modify `sim_state` as needed at the beginning of action execution. Derived classes that
        need to extend start() should typically invoke the parent method.
        """
        sim_state.trace.append(
            TraceEvent(sim_state.elapsed_time, self, self.get_duration())
        )
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
        after_start_time = sim_state.elapsed_time + 0.001
        end_time = sim_state.elapsed_time + self.get_duration()
        self.start(sim_state)
        sim_state.events.push((after_start_time, lambda _: None))
        sim_state.events.push((end_time, self.end))


@dataclass
class RobotState:
    "Represents the state of a robot."

    pos: LocationName
    "The current location of the robot."

    action: Optional[Action]
    "The action the robot is currently executing (or None if it is idle)."

    reserved: List[LocationName]
    """
    A list of locations the robot currently has reserved. (Places the robot is expected to move
    through given the action it is currently executing.)
    """

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {"pos": self.pos, "action": repr(self.action), "reserved": self.reserved}


class PriorityQueue(Generic[T]):
    "Priority queue implemented as a list that maintains the heap invariant."

    def __init__(self, seq: Optional[Iterable[T]] = None):
        ":param seq: Initial contents of the queue (need not be ordered)."
        if seq is None:
            self._q: List[T] = []
        else:
            self._q = list(seq)
            heapq.heapify(self._q)

    def __iter__(self) -> Iterator[T]:
        "Iterate through the queue in order non-destructively. (Not optimized, for debugging.)"
        return iter(sorted(self._q))

    def __bool__(self) -> bool:
        "Return True if the queue is non-empty."
        return bool(self._q)

    def push(self, item: T) -> None:
        "Push a new item onto the queue."
        heapq.heappush(self._q, item)

    def pop(self) -> T:
        "Pop and return the head of the queue in priority order."
        return heapq.heappop(self._q)


@dataclass
class SimState:
    "Represents the overall state of the multi-robot sim."

    robot_states: Dict[RobotName, RobotState]
    "Initial robot states."

    elapsed_time: Timestamp = 0.0
    "Elapsed time since start of simulation (seconds)."

    trace: ExecutionTrace = field(default_factory=list)
    "Trace of timestamped actions executed so far in the sim."

    completed: Set[str] = field(default_factory=set)
    "Tracks completion status for actions that subclass MarkCompleteAction."

    events: PriorityQueue[PendingEvent] = field(default_factory=PriorityQueue)
    "Pending events queued by earlier actions."

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {
            "elapsed_time": self.elapsed_time,
            "robot_states": {
                robot: state.get_dump_dict()
                for robot, state in self.robot_states.items()
            },
            "events": list(self.events),
            "trace": [e.get_dump_dict() for e in self.trace],
            "completed": self.completed,
        }

    def warp(self) -> None:
        """
        Warp the simulation forward in time to the next queued event and apply its event callback.
        AKA "wait for something to happen".
        """
        if not self.events:
            return
        event_time, event_func = self.events.pop()
        self.elapsed_time = event_time
        event_func(self)


@dataclass(repr=False)
class MarkCompleteAction(Action):
    """
    Represents an action that explicitly marks itself complete when it finishes executing.
    """

    order: OrderName
    "Propagates goal ordering constraint from problem instance. Required by PDDL model."

    def get_completed_predicate(self) -> str:
        "Return the completed-x predicate asserted when this action is completed."
        return repr(self).replace("(", "(completed-", 1)

    def end(self, sim_state: SimState) -> None:
        super().end(sim_state)
        sim_state.completed.add(self.get_completed_predicate())

    def is_complete(self, sim_state: SimState) -> bool:
        """
        Return True if this action has been completed in `sim_state`.
        """
        return self.get_completed_predicate() in sim_state.completed


def get_collision_check_locations(
    from_pos: LocationName, to_pos: LocationName, include_to_pos: bool = True
) -> List[str]:
    """
    Return locations that need to be checked for collision avoidance purposes prior to moving to
    `to_pos`. Locations include `to_pos` itself and neighbors of `to_pos` that are distinct from
    `from_pos` and are flying locations. If another robot has reserved any of these locations the
    move is invalid.

    :param include_to_pos: If False, don't include `to_pos` in the result. This is handy
      for filling in move action "check" arguments that conform to the PDDL planning model.
    """
    to_ind = CONFIG.location_lookup[to_pos]
    to_neighbors_ind = CONFIG.neighbors[to_ind]
    to_neighbors = [CONFIG.locations[n] for n in to_neighbors_ind]
    result = [n for n in to_neighbors if is_location_flying(n) and n != from_pos]
    if include_to_pos:
        result.append(to_pos)
    return result


def get_reserved_by_other_robots(
    sim_state: SimState, robot: RobotName
) -> List[LocationName]:
    "Return the list of locations reserved by robots other than `robot` in `sim_state`."
    other_robots = [r for r in CONFIG.robots if r != robot]
    others_reserved: Set[LocationName] = set().union(
        *(
            sim_state.robot_states[robot].reserved for robot in other_robots
        )  # type:ignore
    )
    return list(others_reserved)


def get_collision_reason(
    from_pos: LocationName, to_pos: LocationName, robot: RobotName, sim_state: SimState
) -> str:
    """
    Return the reason why a move from `from_pos` to `to_pos` with `robot` in `sim_state` fails
    collision checking. Return an empty string if the move passes the collision check.
    """
    check_locs = get_collision_check_locations(from_pos, to_pos)
    others_reserved = get_reserved_by_other_robots(sim_state, robot)
    reserved_check_locs = set(others_reserved).intersection(check_locs)
    if reserved_check_locs:
        return (
            f"Expected no collision check locations adjacent to `to_pos` {to_pos} to be reserved by"
            f" other robots, but these are: {reserved_check_locs}"
        )
    return ""  # ok


@dataclass(repr=False)
class AbstractMoveAction(Action):
    "Represents a single-step move action."

    from_pos: LocationName
    "The robot's starting location."

    to: LocationName
    "The location the robot should move to."

    def get_collision_check_locations(self, include_to_pos: bool = True) -> List[str]:
        """
        Return collision check locations for this move action.
        """
        return get_collision_check_locations(
            self.from_pos, self.to, include_to_pos=include_to_pos
        )

    def get_pddl_args(self) -> List[Any]:
        return [self.robot, self.from_pos, self.to]

    def invalid_reason(self, sim_state: SimState) -> str:
        robot_state = sim_state.robot_states[self.robot]
        from_ind = CONFIG.location_lookup[robot_state.pos]
        to_ind = CONFIG.location_lookup[self.to]

        # The robot can only move to an immediate neighbor of its current location
        if to_ind not in CONFIG.neighbors[from_ind]:
            return f"Expected `to` {self.to} to be a neighbor of `from_pos` {self.from_pos}"

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
    "Represents a move action from one flying location to another."

    def get_pddl_args(self) -> List[Any]:
        # One check location argument is required for move
        return (
            super().get_pddl_args()
            + self.get_collision_check_locations(include_to_pos=False)[:1]
        )

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
    "Represents a dock action (from a flying location to a dock berth)."

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
    "Represents an undock action (from a dock berth to a flying location)."

    def get_pddl_args(self) -> List[Any]:
        # Two check location arguments are required for undock
        return (
            super().get_pddl_args()
            + self.get_collision_check_locations(include_to_pos=False)[:2]
        )

    def invalid_reason(self, sim_state: SimState) -> str:
        super_reason = super().invalid_reason(sim_state)
        if super_reason:
            return super_reason
        if not is_location_berth(self.from_pos):
            return f"Expected `from_pos` {self.from_pos} to be a berth"
        if not is_location_flying(self.to):
            return f"Expected `to` {self.to} to be a flying location"
        return ""  # ok


@dataclass(repr=False)
class PanoramaAction(MarkCompleteAction):
    "Represents a panorama action."

    location: LocationName
    "Where to acquire the panorama."

    def get_pddl_args(self) -> List[Any]:
        return [self.robot, self.order, self.location]

    def invalid_reason(self, sim_state: SimState) -> str:
        robot_state = sim_state.robot_states[self.robot]
        if robot_state.pos != self.location:
            return f"Expected robot pos {robot_state.pos} to match panorama pos {self.location}"
        return ""  # ok


@dataclass(repr=False)
class StereoAction(MarkCompleteAction):
    "Represents a stereo survey action."

    base: LocationName
    "The location where the stereo survey starts and ends."

    bound: LocationName
    "The other end of the interval of locations that the robot visits during the survey."

    def get_pddl_args(self) -> List[Any]:
        # Two collision check arguments are required for stereo action
        check_locs = get_collision_check_locations(
            self.base, self.bound, include_to_pos=False
        )[:2]
        return [self.robot, self.order, self.base, self.bound] + check_locs

    def get_completed_predicate(self) -> str:
        # Unlike the panorama action, where the completed-panorama predicate takes exactly the same
        # arguments as the action, for the completed-stereo predicate we don't include the collision
        # check arguments.
        args_str = " ".join(
            ["completed-stereo", self.robot, self.order, self.base, self.bound]
        )
        return f"({args_str})"

    def invalid_reason(self, sim_state: SimState) -> str:
        robot_state = sim_state.robot_states[self.robot]
        if robot_state.pos != self.base:
            return f"Expected robot pos {robot_state.pos} to match stereo survey base {self.base}"

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


@dataclass
class Goal(ABC):
    "Represents an abstract goal."

    robot: RobotName
    "The robot primarily responsible for achieving the goal."

    def is_complete(self, exec_state: "ExecState") -> bool:
        "Return True if the goal has been completed in `exec_state`."
        raise NotImplementedError()  # Note: Can't mark as @abstractmethod due to mypy limitation

    def get_next_action(self, exec_state: "ExecState") -> Action:
        "Return the next action for this robot to apply to achieve the goal given `exec_state`."
        raise NotImplementedError()  # Note: Can't mark as @abstractmethod due to mypy limitation

    def get_visited(self, exec_state: "ExecState") -> List[LocationIndex]:
        "Return the locations to be visited while achieving this goal given `exec_state`."
        raise NotImplementedError()  # Note: Can't mark as @abstractmethod due to mypy limitation

    def get_other_robot_action(
        self, exec_state: "ExecState", robot: RobotName
    ) -> Optional[Action]:
        """
        Return the next action for other `robot` to apply in `exec_state` to proactively avoid
        interfering with achieving this goal. Return None if no proactive action is needed.
        """
        safe_region = get_safe_from(self.get_visited(exec_state))
        safe_goal = MoveToRegionGoal(robot=robot, region=safe_region)
        if safe_goal.is_complete(exec_state):
            return None
        return safe_goal.get_next_action(exec_state)

    def get_other_robot_action_vetoed(  # pylint: disable=unused-argument
        self, exec_state: "ExecState", robot: RobotName, action: Action
    ) -> bool:
        """
        Return True if other `robot` applying `action` is vetoed (becaues it will interfere with
        achieving this goal).
        """
        if isinstance(action, AbstractMoveAction):
            to_ind = CONFIG.location_lookup[action.to]
            return to_ind not in get_safe_from(self.get_visited(exec_state))
        return False


@enum.unique
class RobotStatus(enum.IntEnum):
    "Represents execution status of a robot."

    INIT = enum.auto()
    "Robot has not started execution yet."

    DONE = enum.auto()
    "Idle because robot has completed all of its goals."

    VETOED = enum.auto()
    "Idle because robot's selected action has been vetoed by a higher priority goal from another robot."

    BLOCKED = enum.auto()
    "Idle because robot's selected action is not legal given the current sim state."

    ACTIVE = enum.auto()
    "Actively executing an action."


# Convenience alias so we can write R.DONE instead of RobotStatus.DONE
R = RobotStatus


@dataclass
class RobotExecState:
    "Represents the execution state of a robot."

    goals: List[Goal]
    "Goals of the robot."

    goal_index: int = 0
    """
    Number of goals that the robot has completed. If less than `len(goals)`, this can be
    interpreted as the index of the current goal. (Or if all goals have been completed, there is no
    current goal.)
    """

    goal_start_time: Timestamp = 0.0
    """
    When the most recently completed goal was completed. (Can be interpreted as when the current
    goal became active.) Set to 0.0 if no goal has been completed yet.
    """

    status: RobotStatus = R.INIT
    """
    Execution status of the robot.
    """

    blocked_action: Optional[Action] = None
    """
    If status is R.BLOCKED, this is the next action that is currently invalid. Otherwise, None.
    """

    blocked_reason: str = ""
    """
    If status is R.BLOCKED, this is the reason why `blocked_action` is invalid. Otherwise, "".
    """

    def get_goal(self) -> Optional[Goal]:
        """
        Return the currently active goal, or None if all goals have been achieved.
        """
        if self.goal_index >= len(self.goals):
            return None
        return self.goals[self.goal_index]

    def is_active(self) -> bool:
        "Return True if the robot is actively working on a goal."
        return self.status == R.ACTIVE

    def is_done(self) -> bool:
        "Return True if the robot has achieved all of its goals."
        return self.status == R.DONE

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        result = {
            "goals": [repr(g) for g in self.goals],
            "goal_index": self.goal_index,
            "goal_start_time": self.goal_start_time,
            "status": self.status,
        }
        if self.status == R.BLOCKED:
            result["blocked_action"] = repr(self.blocked_action)
            result["blocked_reason"] = repr(self.blocked_reason)
        return result


def filter_none(seq: Iterable[Optional[T]]) -> List[T]:
    "Return the result of filtering None entries out of `seq`."
    return [elt for elt in seq if elt is not None]


@dataclass
class RobotActionInfo:
    "Represents information about a robot's chosen next action."
    robot: RobotName
    status: RobotStatus
    action: Optional[Action] = None
    blocked_reason: str = ""


@dataclass
class ExecState:
    "Represents the execution state of the multi-robot system."

    sim_state: SimState
    "The initial simulation state."

    robot_exec_states: Dict[RobotName, RobotExecState]
    "The initial execution states of the robots in the multi-robot system."

    robots: List[RobotName]
    "The list of robots in priority order."

    def is_any_robot_active(self) -> bool:
        "Return True if any robot is actively working on a goal."
        return any((rstate.is_active() for rstate in self.robot_exec_states.values()))

    def are_all_robots_done(self) -> bool:
        "Return True if all robots have achieved all of their goals."
        return all((rstate.is_done() for rstate in self.robot_exec_states.values()))

    def skip_completed_goals(self) -> None:
        "Advance robot goal indices past completed goals."
        for robot in CONFIG.robots:
            robot_exec_state = self.robot_exec_states[robot]
            while True:
                robot_goal = robot_exec_state.get_goal()
                if robot_goal is None:
                    break  # Already at end of goals
                if robot_goal.is_complete(self):
                    robot_exec_state.goal_index += 1
                    robot_exec_state.goal_start_time = self.sim_state.elapsed_time
                    continue
                break  # Stop at incomplete goal

    def get_prioritized_goals(self) -> List[Goal]:
        """
        Return a list of active goals, sorted from highest to lowest priority based on the
        priority of the robot that owns the goal.
        """
        return filter_none(
            (self.robot_exec_states[robot].get_goal() for robot in self.robots)
        )

    def is_action_vetoed(
        self, robot: RobotName, action: Action, higher_priority_goals: List[Goal]
    ) -> bool:
        """
        Return True if `robot` can't execute `action` because a goal in `higher_priority_goals`
        vetoes it.
        """
        return any(
            (
                goal.get_other_robot_action_vetoed(self, robot, action)
                for goal in higher_priority_goals
            )
        )

    def get_next_action(
        self, prioritized_goals: List[Goal], robot: RobotName
    ) -> RobotActionInfo:
        """
        Return info about the chosen next action for `robot` given `prioritized_goals`.
        """
        action: Optional[Action] = None
        for goal_index, goal in enumerate(prioritized_goals):
            if goal.robot == robot:
                action = goal.get_next_action(self)
            else:
                action = goal.get_other_robot_action(self, robot)
            if action is not None:
                break

        if action is None:
            return RobotActionInfo(robot, R.DONE)

        # pylint: disable-next=undefined-loop-variable
        higher_priority_goals = prioritized_goals[:goal_index]
        if self.is_action_vetoed(robot, action, higher_priority_goals):
            return RobotActionInfo(robot, R.VETOED, action)

        blocked_reason = action.invalid_reason(self.sim_state)
        if blocked_reason:
            return RobotActionInfo(
                robot, R.BLOCKED, action, blocked_reason=blocked_reason
            )

        return RobotActionInfo(robot, R.ACTIVE, action)

    def get_next_actions(self, prioritized_goals: List[Goal]) -> List[RobotActionInfo]:
        """
        Return info about chosen next actions for robots not currently executing an action, given
        `prioritized_goals`.
        """
        next_actions = [
            self.get_next_action(prioritized_goals, robot)
            for robot in self.robots
            if self.sim_state.robot_states[robot].action is None
        ]

        already_applied_action = False
        for next_action in next_actions:
            if next_action.status == R.ACTIVE:
                if already_applied_action:
                    next_action.status = R.BLOCKED
                    next_action.blocked_reason = "PDDL temporal logic rules forbid starting multiple actions at the same moment"
                else:
                    already_applied_action = True

        return next_actions

    def apply_next_action(self, next_actions: List[RobotActionInfo]) -> None:
        """
        Apply chosen next action for one robot and update all robot execution states as needed.
        """
        for action_info in next_actions:
            robot_exec_state = self.robot_exec_states[action_info.robot]
            robot_exec_state.status = action_info.status
            if action_info.status == R.ACTIVE:
                assert action_info.action is not None
                action_info.action.apply(self.sim_state)
            if action_info.status == R.BLOCKED:
                robot_exec_state.blocked_action = action_info.action
                robot_exec_state.blocked_reason = action_info.blocked_reason
            else:
                robot_exec_state.blocked_action = None
                robot_exec_state.blocked_reason = ""

    def run_step(self) -> None:
        "Try to apply each robot's next action if it is idle."
        self.skip_completed_goals()
        prioritized_goals = self.get_prioritized_goals()
        next_actions = self.get_next_actions(prioritized_goals)
        self.apply_next_action(next_actions)

    def get_dump_dict(self) -> Dict[str, Any]:
        "Return a dict to dump for debugging."
        return {
            "sim_state": self.sim_state.get_dump_dict(),
            "robot_exec_states": {
                robot: state.get_dump_dict()
                for robot, state in self.robot_exec_states.items()
            },
            "robots": self.robots,
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
                print(self)
                raise RuntimeError(
                    "Can't achieve all goals. Not done but no active robots!"
                )


def get_move_action(
    robot: RobotName, from_pos: LocationName, to_pos: LocationName
) -> AbstractMoveAction:
    "Return the single-step move action required to move `robot` from `from_pos` to `to_pos`."
    if is_location_berth(to_pos):
        action_type: Type[AbstractMoveAction] = DockAction
    elif is_location_berth(from_pos):
        action_type = UndockAction
    else:
        action_type = MoveAction
    return action_type(robot=robot, from_pos=from_pos, to=to_pos)


@dataclass
class MoveToRegionGoal(Goal):
    "Represents a goal of moving to any location in the specified region."

    region: List[LocationIndex]
    "The region the robot should move to."

    def __repr__(self) -> str:
        space_str = " ".join((CONFIG.locations[l] for l in self.region))
        return f"(move-region {self.robot} [{space_str}])"

    def is_complete(self, exec_state: ExecState) -> bool:
        "Return True if the robot is already in the desired region in `exec_state`."
        robot_state = exec_state.sim_state.robot_states[self.robot]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        return pos_ind in self.region

    def get_next_action(self, exec_state: ExecState) -> Action:
        "Return a single-step move action toward the desired region from `exec_state`."
        robot_state = exec_state.sim_state.robot_states[self.robot]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        blocked_region = [
            CONFIG.location_lookup[loc]
            for loc in get_reserved_by_other_robots(exec_state.sim_state, self.robot)
        ]
        next_ind = get_move_location(
            goal_region=self.region,
            blocked_region=blocked_region,
            current_pos=pos_ind,
        )
        next_loc = CONFIG.locations[next_ind]
        return get_move_action(self.robot, robot_state.pos, next_loc)

    def get_visited(self, exec_state: ExecState) -> List[LocationIndex]:
        robot_state = exec_state.sim_state.robot_states[self.robot]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        blocked_region = [
            CONFIG.location_lookup[loc]
            for loc in get_reserved_by_other_robots(exec_state.sim_state, self.robot)
        ]
        return get_visited(
            from_pos=pos_ind, to_region=self.region, blocked_region=blocked_region
        )


@dataclass
class MoveGoal(Goal):
    "Represents a move goal (may require multiple single-step move actions)."

    to: LocationName
    "The location the robot should move to."

    def __repr__(self) -> str:
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
        return get_move_action(self.robot, robot_state.pos, next_loc)

    def get_visited(self, exec_state: ExecState) -> List[LocationIndex]:
        robot_state = exec_state.sim_state.robot_states[self.robot]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        to_ind = CONFIG.location_lookup[self.to]
        return get_visited(from_pos=pos_ind, to_region=[to_ind])


@dataclass
class MarkCompleteGoal(Goal):
    "Represents a goal that is complete when a corresponding MarkCompleteAction finishes."

    order: OrderName
    "Propagates goal ordering constraint from problem instance. Required by PDDL model."

    def get_completing_action(self) -> MarkCompleteAction:
        "Return the MarkCompleteAction whose completion satisfies this MarkCompleteGoal."
        raise NotImplementedError()  # Note: Can't mark as @abstractmethod due to mypy limitation

    def __repr__(self) -> str:
        return self.get_completing_action().get_completed_predicate()

    def is_complete(self, exec_state: ExecState) -> bool:
        return self.get_completing_action().is_complete(exec_state.sim_state)


@dataclass
class PanoramaGoal(MarkCompleteGoal):
    "Represents a panorama goal."

    location: LocationName
    "The location where the panorama should be acquired."

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

    def get_visited(self, exec_state: ExecState) -> List[LocationIndex]:
        robot_state = exec_state.sim_state.robot_states[self.robot]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        to_ind = CONFIG.location_lookup[self.location]
        return get_visited(from_pos=pos_ind, to_region=[to_ind])


@dataclass
class StereoGoal(MarkCompleteGoal):
    "Represents a stereo survey goal."

    base: LocationName
    "The location where the stereo survey starts and ends."

    bound: LocationName
    "The other end of the interval of locations that the robot visits during the survey."

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

    def get_visited(self, exec_state: ExecState) -> List[LocationIndex]:
        robot_state = exec_state.sim_state.robot_states[self.robot]
        pos_ind = CONFIG.location_lookup[robot_state.pos]
        base_ind = CONFIG.location_lookup[self.base]
        bound_ind = CONFIG.location_lookup[self.bound]
        result = set()
        result.update(get_visited(from_pos=pos_ind, to_region=[base_ind]))
        result.update(get_visited(from_pos=base_ind, to_region=[bound_ind]))
        return list(result)


def format_trace(trace: ExecutionTrace) -> str:
    "Return `trace` formatted in the standard PDDL plan output format used by POPF."
    out = io.StringIO()
    for event in trace:
        print(f"{event.timestamp:.3f}: {event.action} [{event.duration:.3f}]", file=out)
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
        except ValueError as exc:
            raise RuntimeError(
                f"Expected duration value to be a float, got {repr(duration_str)}"
            ) from exc
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
        f"WARNING: Can't map PDDL goal_type {goal_type} to custom planner goal type, ignoring",
        file=sys.stderr,
    )
    return None


def get_objects_by_type(
    problem: PddlExpression,
) -> Dict[PddlTypeName, List[PddlObjectName]]:
    """
    Return mapping from PDDL type name to a list of object instances of that type, parsed from
    `problem`.
    """
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


@dataclass
class RobotGoalsPrio:
    """
    Represents information from PDDL problem instance goals.
    """

    robot_goals: Dict[RobotName, List[Goal]]
    "Maps robot name to its goals."

    robots: List[RobotName]
    "List of robots prioritized from highest to lowest."


def get_robot_goals_prio(problem: PddlExpression) -> RobotGoalsPrio:
    """
    Return a mapping of robot name to robot goal info parsed from `problem`.
    """
    (goal_clause,) = [e for e in problem[2:] if e[0] == ":goal"]
    compound_expr = goal_clause[1]
    if compound_expr[0] == "and":
        goal_exprs = compound_expr[1:]
    else:
        goal_exprs = [compound_expr]
    goals = filter_none([get_goal_from_pddl(goal_expr) for goal_expr in goal_exprs])

    robot_first_mention: Dict[RobotName, int] = {}
    for goal_index, goal in enumerate(goals):
        robot_first_mention.setdefault(goal.robot, goal_index)
    lowest_priority = len(goals)

    prio_robot_pairs = [
        (robot_first_mention.get(robot, lowest_priority), robot)
        for robot in CONFIG.robots
    ]
    prioritized_robots = [robot for _, robot in sorted(prio_robot_pairs)]

    robot_goals = {
        robot: [g for g in goals if g.robot == robot] for robot in prioritized_robots
    }
    return RobotGoalsPrio(robot_goals, prioritized_robots)


def get_init_predicates(problem: PddlExpression) -> List[PddlPredicate]:
    "Return the predicates in the initial state of `problem`."
    # Example problem snippet:
    # (define (problem jem-survey)
    #  ... other clauses ...
    #  (:init (pred1 arg1 arg2) (pred2 arg1 arg2)))
    #
    # We want to return the list of predicates in the :init clause.
    (init_clause,) = [e for e in problem[2:] if e[0] == ":init"]
    return init_clause[1:]


def get_robot_states(problem: PddlExpression) -> Dict[RobotName, RobotState]:
    """
    Return a mapping of robot name to robot state extracted from `problem`.
    """
    robot_states = {
        robot: RobotState(pos="", action=None, reserved=[]) for robot in CONFIG.robots
    }
    init_predicates = get_init_predicates(problem)
    robot_at_predicates = [p for p in init_predicates if p[0] == "robot-at"]
    for _, robot, pos in robot_at_predicates:
        robot_states[robot].pos = pos
        robot_states[robot].reserved = [pos]
    return robot_states


def get_completed_predicates(problem: PddlExpression) -> List[PddlPredicate]:
    "Return the completed action predicates in the initial state of `problem`."
    return [p for p in get_init_predicates(problem) if p[0].startswith("completed-")]


def string_from_predicate(expr: PddlPredicate) -> str:
    "Return the PDDL string representation of `expr`."
    space_list = " ".join((str(item) for item in expr))
    return f"({space_list})"


def survey_planner(domain_path: pathlib.Path, problem_path: pathlib.Path) -> None:
    "Primary driver function for custom planning."

    domain_expr = parse_pddl(domain_path)
    CONFIG.action_durations = get_action_durations(domain_expr)

    problem_expr = parse_pddl(problem_path)
    CONFIG.robots = get_objects_by_type(problem_expr)["robot"]

    location_config = get_location_config(problem_expr)
    CONFIG.locations = location_config["locations"]
    CONFIG.location_lookup = location_config["location_lookup"]
    CONFIG.neighbors = location_config["neighbors"]

    goals_prio = get_robot_goals_prio(problem_expr)
    robot_states = get_robot_states(problem_expr)
    completed_predicates = get_completed_predicates(problem_expr)
    completed_set = {string_from_predicate(p) for p in completed_predicates}

    sim_state = SimState(robot_states=robot_states, completed=completed_set)
    robot_exec_states = {}
    for robot in CONFIG.robots:
        robot_exec_states[robot] = RobotExecState(goals=goals_prio.robot_goals[robot])
    exec_state = ExecState(
        sim_state=sim_state,
        robot_exec_states=robot_exec_states,
        robots=goals_prio.robots,
    )

    exec_state.run()
    print("; Solution Found")
    print(format_trace(exec_state.sim_state.trace))


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter, argparse.RawDescriptionHelpFormatter
):
    "Custom formatter for argparse that combines mixins."


def main() -> int:
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
