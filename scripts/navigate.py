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
    pass


@dataclass
class Orient:
    pass


@dataclass
class Drive:
    pass


State = Union[
    Wait,
    Orient,
    Drive,
]


@dataclass
class Model:
    path: List[Cell]
    state: State


init_model: Model = Model(
    path=[Cell(7, 9)],
    state=Orient(),
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Odom:
    pose: TurtlePose


Msg = Union[Odom]


### Update ###

DRIVE_VEL_LIN: float = 0.6
DRIVE_VEL_ANG_MAX: float = 1.5 * math.pi

GRID_SIDE_LEN: float = 14.754
GRID_NUM_CELLS: int = 13
GRID_ORIGIN: Vector2 = v2.scale(Vector2(GRID_SIDE_LEN, GRID_SIDE_LEN), -0.5)

GRID: Grid = Grid(
    len_cell=GRID_SIDE_LEN / GRID_NUM_CELLS,
    origin=GRID_ORIGIN,
)


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(model.state, Wait):
        return (model, [cmd.stop])

    if (next_cell := head(model.path)) is None:
        return (transition(model, state=Wait()), cmd.none)

    to_cell = grid.next_cell_direction(GRID, msg.pose, next_cell)
    err_ang = to_cell / math.pi

    if isinstance(model.state, Orient):
        if approx_zero(err_ang):
            return (transition(model, state=Drive()), [cmd.stop])

        vel_ang = lerp_signed(0.05 * math.pi, 1.0 * math.pi, err_ang)

        return (model, [cmd.turn(vel_ang)])

    cell_offset = grid.current_cell_offset(GRID, msg.pose)
    err_lin = cell_offset / GRID.len_cell

    err = 0.5 * (err_ang + err_lin)

    vel_ang = lerp_signed(low=0, high=DRIVE_VEL_ANG_MAX, amount=err)
    vel_lin = mathf.lerp(low=0.6, high=0.2, amount=10.0 * abs(err))

    return (model, [cmd.velocity(angular=vel_ang, linear=vel_lin)])


def transition(model: Model, state: State) -> Model:
    print(f"Begin {state}")
    return replace(model, state=state)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, Msg]]:
    return [sub.odometry(Odom)]


### Run ###


def run() -> None:
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
