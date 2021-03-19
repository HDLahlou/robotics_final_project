#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
import math
from typing import Any, List, Tuple, Union

import rospy
import rospy_util.mathf as mathf
from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2

from controller import Cmd, Controller, Sub, cmd, sub
from navigation.grid import Cell, Grid
import navigation.grid as grid
from util import approx_zero, head, lerp_signed

### Model ###


@dataclass
class Wait:
    """
    Wait for directions.
    """

    reason: str


@dataclass
class Orient:
    """
    Turn to face the next cell.
    """

    pass


@dataclass
class Drive:
    """
    Move forward to the next cell.
    """

    pass


@dataclass
class Turn:
    """
    Turn to reach the next cell.
    """

    pass


State = Union[
    Wait,
    Orient,
    Drive,
    Turn,
]


@dataclass
class Model:
    """
    Model of the robot.

    @attribute `state`: Current state of the robot.

    @attribute `path`: Path to take to the destination.
    """

    state: State
    path: List[Cell]


init_model: Model = Model(
    path=[],
    state=Wait("Awaiting directions..."),
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Odom:
    """
    Pose of the robot as estimated by odometry.
    """

    pose: TurtlePose


@dataclass
class Directions:
    """
    Path for the robot to follow.
    """

    path: List[Cell]


Msg = Union[
    Odom,
    Directions,
]


### Update ###

DRIVE_VEL_LIN_MAX: float = 0.6
DRIVE_VEL_LIN_MIN: float = 0.6
DRIVE_VEL_ANG_MAX: float = 1.5 * math.pi

TURN_VEL_LIN: float = 0.5
TURN_VEL_ANG: float = 1.8 * math.pi

GRID_SIDE_LEN: float = 14.754
GRID_NUM_CELLS: int = 13
GRID_ORIGIN: Vector2 = v2.scale(Vector2(GRID_SIDE_LEN, GRID_SIDE_LEN), -0.5)

GRID: Grid = Grid(
    len_cell=GRID_SIDE_LEN / GRID_NUM_CELLS,
    origin=GRID_ORIGIN,
)


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    """
    Given an inbound message and the current robot model, produce a new model
    and a list of commands to execute.
    """
    if isinstance(msg, Directions):
        if not model.path:
            if not grid.path_is_valid(msg.path):
                print("Path is invalid!")
                return (model, cmd.none)

            new_model = replace(model, path=msg.path)
            return (transition(new_model, state=Orient()), cmd.none)

        print("Already have directions!")
        return (model, cmd.none)

    if isinstance(model.state, Wait):
        return (model, [cmd.stop])

    if (next_cell := head(model.path)) is None:
        return wait(model, reason="Path is empty")

    current_cell = grid.locate_pose(GRID, msg.pose)

    crossing_cells = current_cell == next_cell

    if crossing_cells:
        # this is illegal
        model = replace(model, path=model.path[1:])

        #  this is also illegal
        if (next_cell := head(model.path)) is None:
            return wait(model, reason="Path traversed")

        print(f"next cell is {next_cell}")

    if not grid.cells_are_adjacent(current_cell, next_cell):
        return wait(model, reason="Next cell not adjacent")

    to_cell = grid.next_cell_direction(
        cell_current=current_cell,
        cell_next=next_cell,
        pose=msg.pose,
    )

    err_ang = to_cell / math.pi

    if isinstance(model.state, Orient):
        if approx_zero(err_ang):
            return (transition(model, state=Drive()), [cmd.stop])

        vel_ang = lerp_signed(0.05 * math.pi, 1.0 * math.pi, err_ang)

        return (model, [cmd.turn(vel_ang)])

    if isinstance(model.state, Drive):
        if abs(err_ang) > 0.25:
            return (transition(model, state=Turn()), cmd.none)

        cell_offset = grid.current_cell_offset(
            grid=GRID,
            cell_current=current_cell,
            cell_next=next_cell,
            pose=msg.pose,
        )

        err_lin = cell_offset / GRID.len_cell

        err = 0.5 * (err_ang + err_lin)

        vel_ang = lerp_signed(
            low=0,
            high=DRIVE_VEL_ANG_MAX,
            amount=err,
        )

        vel_lin = mathf.lerp(
            low=DRIVE_VEL_LIN_MAX,
            high=DRIVE_VEL_LIN_MIN,
            amount=10.0 * abs(err),
        )

        return (model, [cmd.velocity(angular=vel_ang, linear=vel_lin)])

    if approx_zero(err_ang):
        return (transition(model, state=Drive()), cmd.none)

    vel_ang = lerp_signed(0, TURN_VEL_ANG, err_ang)

    return (model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])


def wait(model: Model, reason: str) -> Tuple[Model, List[Cmd[Any]]]:
    """
    Transition the bot to a wait state.
    """
    return (transition(model, state=Wait(reason)), cmd.none)


def transition(model: Model, state: State) -> Model:
    """
    Transition to a new state while printing a notification.
    """
    print(f"Begin {state}")
    return replace(model, state=state)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, Msg]]:
    """
    Subscriptions from which to receive messages.
    """
    return [
        sub.odometry(Odom),
        sub.directions(Directions),
    ]


### Run ###


def run() -> None:
    """
    Run the navigation module.
    """
    rospy.init_node("laser_navigate")

    # Run the ROS controller

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
