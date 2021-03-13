#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Any, List, Tuple, TypeVar, Union

import cv_bridge, cv2
import numpy as np
import rospy
import rospy_util.mathf as mathf
from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2
from sensor_msgs.msg import Image, LaserScan

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


@dataclass
class Spin:
    angle: float


Model = Union[
    Wait,
    Turn,
    Drive,
    Spin,
]


init_model: Model = Drive()

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Scan:
    ranges: List[Range]


@dataclass
class Odom:
    pose: TurtlePose


Msg = Union[
    Scan,
    Odom,
]


### Update ###

DRIVE_VEL_LIN: float = 0.6
DRIVE_KP_ANG: float = 3.0

TURN_START_DIST: float = 2.0
TURN_END_DIST: float = 1.0

TURN_VEL_LIN: float = 0.65
TURN_KP_ANG: float = 1.1

SPIN_KP_ANG: float = 1.0


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(model, Wait):
        return (model, cmd.none)

    if isinstance(msg, Scan):
        print(msg.ranges[0])
        if msg.ranges[0].dist < 0.5:
            print("spin")
            return (Spin(0), [cmd.stop])

        if isinstance(model, Drive):
            left_max = max(msg.ranges[45:75], key=lambda r: r.dist)

            if left_max.dist > TURN_START_DIST:
                print("turn left")
                return (Turn(Direction.LEFT), cmd.none)

            right_max = max(msg.ranges[285:315], key=lambda r: r.dist)

            if right_max.dist > TURN_START_DIST:
                print("turn right")
                return (Turn(Direction.RIGHT), cmd.none)

            left_min = min(msg.ranges[45:135], key=lambda r: r.dist)
            right_min = min(msg.ranges[225:315], key=lambda r: r.dist)

            err_lin = min(left_min.dist - right_min.dist, 1.0)
            err_ang = ((left_min.dir - 90) / 90) + ((right_min.dir - 270) / 90)

            err = 0.5 * err_lin + 0.5 * err_ang

            vel_ang = mathf.sign(err) * mathf.lerp(
                low=0, high=DRIVE_KP_ANG, amount=abs(err)
            )

            print(vel_ang)

            return (model, [cmd.velocity(angular=vel_ang, linear=DRIVE_VEL_LIN)])

        if isinstance(model, Turn):
            vel_ang = (1.0 if model.dir == Direction.LEFT else -1.0) * TURN_KP_ANG
            ranges_side = (
                msg.ranges[60:80]
                if model.dir == Direction.LEFT
                else msg.ranges[280:300]
            )
            range_front = msg.ranges[0]

            if (
                all([r.dist < TURN_END_DIST for r in ranges_side])
                and range_front.dist > 1.5
            ):
                print("drive")
                return (Drive(), cmd.none)

            return (model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])

        return (model, cmd.none)

    # if isinstance(msg, Odom):

    if isinstance(model, Spin):
        err_ang = v2.signed_angle_between(
            v2.from_angle(msg.pose.yaw),
            v2.from_angle(model.angle),
        )

        print("err_ang: ", err_ang)

        if abs(err_ang) < 1e-2:
            return (Drive(), cmd.none)

        return (model, [cmd.turn(SPIN_KP_ANG * err_ang)])

    return (model, cmd.none)


def delta_angle(pose: TurtlePose, target: Vector2) -> float:
    """
    Compute the angle between the direction of the TurtleBot and the target direction.
    """
    direction_facing = v2.from_angle(pose.yaw)
    return v2.signed_angle_between(target, direction_facing)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Any, Msg]]:
    return [
        sub.laser_scan(angled_ranges),
        sub.odometry(Odom),
    ]


def angled_ranges(scan: LaserScan) -> Msg:
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
