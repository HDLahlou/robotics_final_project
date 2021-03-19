#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

import rospy
import rospy_util.mathf as mathf
from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2

from controller import Cmd, Controller, Sub, cmd, sub
from navigation.grid import Cell, Grid
import navigation.grid as grid
from robotics_final_project.msg import PathQuery
from perception import CvBridge, ImageBGR, ImageROS, image, light
from util import approx_zero, compose, lerp_signed

### Model ###


@dataclass
class Wait:
    """
    Wait for directions.
    """

    reason: str


@dataclass
class FaceCell:
    """
    Turn to face the next cell.
    """

    pass


@dataclass
class ApproachCell:
    """
    Turn to face the next cell.
    """

    pass


@dataclass
class FaceHeading:
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
    FaceCell,
    ApproachCell,
    FaceHeading,
    Drive,
    Turn,
]


@dataclass
class Model:
    """
    Model of the robot.

    @attribute `state`: Current state of the robot.

    @attribute `path`: Path to take to the destination.

    @attribute `heading`: Direction the robot is facing upon entering a cell.
    """

    cv_bridge: CvBridge
    state: State
    path: List[Cell]
    headings: List[Vector2]
    current_cell: Optional[Cell]
    player_cell: Optional[Cell]


CELL_DEST: Cell = Cell(8, 6)

init_model: Model = Model(
    cv_bridge=CvBridge(),
    state=Wait("Awaiting path..."),
    path=[],
    headings=[],
    current_cell=None,
    player_cell=None,
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


@dataclass
class Image:
    """
    Image taken by the robot camera.
    """

    image: ImageBGR


@dataclass
class Player:
    """"""

    cell: Cell


Msg = Union[
    Odom,
    Player,
    Directions,
    Image,
]


### Update ###

DRIVE_VEL_LIN_MAX: float = 0.6
DRIVE_VEL_LIN_MIN: float = 0.4
DRIVE_VEL_ANG_MAX: float = 1.5 * math.pi
DRIVE_REORIENT_ERR: float = 0.3

TURN_VEL_LIN: float = 0.5
TURN_VEL_ANG: float = 1.8 * math.pi

GRID_SIDE_LEN: float = 14.754
GRID_NUM_CELLS: int = 13
GRID_ORIGIN: Vector2 = Vector2(-0.5 * GRID_SIDE_LEN, -0.5 * GRID_SIDE_LEN)

GRID: Grid = Grid(
    len_cell=GRID_SIDE_LEN / GRID_NUM_CELLS,
    origin=GRID_ORIGIN,
)

LIGHT_VAL_MIN: float = 50


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    """
    Given an inbound message and the current robot model, produce a new model
    and a list of commands to execute.
    """

    if isinstance(msg, Directions):
        if len(msg.path) < 2:
            print(f"WARN: Received path of length {len(msg.path)}!")
            return (model, cmd.none)

        if not (headings := grid.validate_path(msg.path)):
            print("WARN: Not all cells are adjacent in path!")
            return (model, cmd.none)

        # If the path is valid and the bot isn't already traversing one,
        # orient toward the next cell

        new_state = (
            model.state
            if len(model.path) > 1 and model.path[1] == msg.path[1]
            else FaceCell()
        )

        new_model = replace(model, path=msg.path, headings=headings)

        return (transition(new_model, state=new_state), cmd.none)

    if isinstance(msg, Player) and model.current_cell is not None:
        # Send a request to the A* algorithm when the player robot changes cells
        query_cmd: List[Cmd[PathQuery]] = (
            [
                cmd.request_path(
                    start=model.current_cell,
                    end=msg.cell,
                    blocked=[],
                    # TODO
                    direction=model.headings[0] if model.headings else v2.up,
                )
            ]
            if msg.cell != model.player_cell
            else cmd.none
        )

        return (replace(model, player_cell=msg.cell), query_cmd)

    if isinstance(msg, Odom):
        current_cell = grid.locate_pose(GRID, msg.pose)

        if isinstance(model.state, Wait):
            return (replace(model, current_cell=current_cell), [cmd.stop])

        # If the bot has crossed into a new cell, update the model
        crossing = current_cell == model.path[1]

        (path, headings) = (
            (model.path[1:], model.headings[1:])
            if crossing
            else (model.path, model.headings)
        )

        if len(path) < 2:
            # If the bot has finished traversing the path, stop
            return wait(model, reason="Reached end of path", path=[])

        next_cell = path[1]

        new_model = replace(
            model,
            path=path,
            headings=headings,
            current_cell=current_cell,
        )

        if crossing and not v2.equals(model.headings[0], headings[0]):
            # If the bot is crossing into a new cell and needs to turn
            return (transition(new_model, state=Turn()), cmd.none)

        # Calculate the angular offset between the robot's yaw and the direction
        # of the next cell
        angle_off = v2.signed_angle_between(v2.from_angle(msg.pose.yaw), headings[0])
        err_ang = angle_off / math.pi

        # Center the robot on the cell it's in before beginning path navigation
        start_pos = grid.locate_cell(GRID, new_model.path[0])
        to_start = start_pos - msg.pose.position

        start_err_lin = v2.magnitude(to_start) / (0.5 * GRID.len_cell)
        start_err_ang = (
            v2.signed_angle_between(v2.from_angle(msg.pose.yaw), to_start) / math.pi
        )

        if isinstance(new_model.state, FaceCell):
            if approx_zero(start_err_ang):
                return (transition(new_model, state=ApproachCell()), cmd.none)

            vel_ang = lerp_signed(
                low=0.05 * math.pi,
                high=1.0 * math.pi,
                amount=start_err_ang,
            )

            return (new_model, [cmd.turn(vel_ang)])

        if isinstance(new_model.state, ApproachCell):
            if approx_zero(start_err_lin, epsilon=0.2):
                return (transition(new_model, state=FaceHeading()), cmd.none)

            vel_ang = lerp_signed(
                low=0.0,
                high=2.0 * math.pi,
                amount=start_err_ang,
            )

            vel_lin = mathf.lerp(
                low=0.05,
                high=0.3,
                amount=start_err_lin,
            )

            return (
                new_model,
                [cmd.velocity(angular=start_err_ang, linear=vel_lin)],
            )

        if isinstance(new_model.state, FaceHeading):
            # If Orient, spin until it is facing the next cell
            if approx_zero(err_ang):
                return (transition(new_model, state=Drive()), [cmd.stop])

            vel_ang = lerp_signed(0.05 * math.pi, 1.0 * math.pi, err_ang)

            return (new_model, [cmd.turn(vel_ang)])

        if isinstance(new_model.state, Drive):
            # If Drive, move forward toward the center of the next cell
            if abs(err_ang) > DRIVE_REORIENT_ERR:
                # If the robot isn't facing the path, reorient it
                return (transition(new_model, state=FaceHeading()), [cmd.stop])

            cell_offset = grid.current_cell_offset(
                grid=GRID,
                cell_current=current_cell,
                cell_next=next_cell,
                pose=msg.pose,
            )

            # Calculate the linear offset between the robot's position and
            # the center of the cell it is traveling through
            err_lin = cell_offset / GRID.len_cell
            err = 0.5 * (err_ang + err_lin)

            vel_ang = lerp_signed(
                low=0,
                high=DRIVE_VEL_ANG_MAX,
                amount=err,
            )

            # Decrease the linear velocity if the robot is too close to a wall
            vel_lin = mathf.lerp(
                low=DRIVE_VEL_LIN_MAX,
                high=DRIVE_VEL_LIN_MIN,
                amount=10.0 * abs(err),
            )

            return (new_model, [cmd.velocity(angular=vel_ang, linear=vel_lin)])

        # if isinstance(model.state, Turn):
        if approx_zero(err_ang):
            return (transition(new_model, state=Drive()), cmd.none)

        vel_ang = lerp_signed(0, TURN_VEL_ANG, err_ang)

        return (new_model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])

    if (
        isinstance(msg, Image)
        and len(model.path) > 1
        and model.player_cell is not None
        and (isinstance(model.state, Drive) or isinstance(model.state, Turn))
    ):
        # Compute value of brightest area in image.
        (val_max, _) = light.locate_brightest(msg.image)

        if val_max >= LIGHT_VAL_MIN:
            # Value is sufficiently high; consider the detected object a light
            # and request a new path.

            blocked_cell = model.path[1]

            if blocked_cell == model.player_cell:
                return wait(model, reason="Destination blocked...")

            cmd_query = cmd.request_path(
                start=model.path[0],
                end=model.player_cell,
                blocked=[blocked_cell],
                direction=model.headings[0],
            )

            return (
                transition(model, Wait(reason="Light detected, recalculating path")),
                [cmd_query],
            )

        return (model, cmd.none)

    return (model, cmd.none)


def wait(model: Model, reason: str, **kwargs: Any) -> Tuple[Model, List[Cmd[Any]]]:
    """
    Transition the bot to a wait state.
    """
    return (transition(model, state=Wait(reason), **kwargs), cmd.none)


def transition(model: Model, state: State, **kwargs: Any) -> Model:
    """
    Transition to a new state while printing a notification.
    """
    print(f"Begin {state}")
    return replace(model, state=state, **kwargs)


def to_image_cv2(cvBridge: CvBridge) -> Callable[[ImageROS], Msg]:
    """
    Convert a ROS image message to a CV2 image in BGR format.
    """
    return compose(Image, partial(image.from_ros_image, cvBridge))


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    """
    Subscriptions from which to receive messages.
    """
    return [
        sub.odometry("hunter", Odom),
        sub.directions(Directions),
        sub.image_sensor(to_image_cv2(model.cv_bridge)),
        sub.odometry("player", player_cell),
    ]


def player_cell(pose: TurtlePose) -> Msg:
    return Player(grid.locate_position(GRID, pose.position))


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
