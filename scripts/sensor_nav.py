#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Any, Callable, List, Tuple, TypeVar, Union

import more_itertools as mit
import rospy
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2
from sensor_msgs.msg import LaserScan

from controller import Cmd, Controller, Sub, cmd, sub

T = TypeVar("T")


@dataclass
class Range:
    dir: int
    dist: float


class Direction(IntEnum):
    LEFT = 1
    RIGHT = 2


### Model ###


@dataclass
class Wait:
    pass


@dataclass
class Drive:
    pass


@dataclass
class Turn:
    dir: Direction


Model = Union[
    Wait,
    Turn,
    Drive,
]


init_model: Model = Drive()

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Scan:
    ranges: List[Range]


Msg = Union[Scan]


### Update ###

TURN_START_DIST: float = 2.0
TURN_END_DIST: float = 0.7


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    print(model)

    if isinstance(model, Wait):
        return (model, cmd.none)

    if isinstance(model, Drive):
        range_left = max(msg.ranges[60:90], key=lambda r: r.dist)

        if range_left.dist > TURN_START_DIST:
            return (Turn(Direction.LEFT), cmd.none)

        range_right = max(msg.ranges[270:300], key=lambda r: r.dist)

        if range_right.dist > TURN_START_DIST:
            return (Turn(Direction.RIGHT), cmd.none)

        return (model, [cmd.drive(0.3)])

    vel_ang = (1.0 if model.dir == Direction.LEFT else -1.0) * 0.15 * math.pi
    ranges_side = (
        msg.ranges[70:90] if model.dir == Direction.LEFT else msg.ranges[270:290]
    )
    range_front = msg.ranges[0]

    print(model.dir)

    if all([r.dist < TURN_END_DIST for r in ranges_side]) and range_front.dist > 2.5:
        return (Drive(), cmd.none)

    return (model, [cmd.velocity(angular=vel_ang, linear=0.35)])

    # ranges = [*mit.split_at(msg.ranges, lambda r: r.dist > 0.75)]

    # if len(sanitized := [rs for rs in ranges if rs]) < 2:
    #     return (model, cmd.none)

    # rejoined = (
    #     [sanitized[-1] + sanitized[0], *sanitized[1:-1]] if ranges[-1] else sanitized
    # )

    # (wall_1, _, *_) = sorted(rejoined, key=len)

    # print(points_from_ranges(wall_1))

    # max_right = max(msg.ranges[270:300], key=lambda r: r.dist)
    # max_left = max(msg.ranges[60:90], key=lambda r: r.dist)

    # if max_right.dist > 2.0:
    #     print("corner right")

    # if max_left.dist > 2.0:
    #     print("corner left")

    # vel_ang = 0.3 * min(max_left.dist, 2.5 * math.pi) if max_left.dist > 2.0 else 0.0

    # return (model, [cmd.velocity(angular=vel_ang, linear=0.6)])


def ranges_under(dist_min: float, rs: List[Range]) -> List[Range]:
    return [r for r in rs if r.dist < dist_min]


def points_from_ranges(rs: List[Range]) -> List[Vector2]:
    return [v2.scale(v2.from_angle(r.dir), r.dist) for r in rs]


def split(xs: List[T], pred: Callable[[T], bool]) -> List[List[T]]:
    return [ys for ys in mit.split_at(xs, pred) if ys]


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, Msg]]:
    return [sub.laser_scan(angled_ranges)]


def angled_ranges(scan: LaserScan) -> Msg:
    # def rad_signed(deg: int) -> float:
    #     return math.radians(deg - 360 if deg > 180 else deg)

    return Scan([Range(deg, dist) for (deg, dist) in enumerate(scan.ranges)])


### Run ###


def run() -> None:
    rospy.init_node("q_learning_robot_action")

    # Run the ROS controller

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
