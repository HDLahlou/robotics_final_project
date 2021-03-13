#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
import random
from typing import Any, Callable, List, Optional, Tuple, TypeVar, Union

import rospy
from rospy_util.turtle_pose import TurtlePose
import rospy_util.vector2 as v2
from sensor_msgs.msg import Image, LaserScan

from controller import Cmd, Controller, Sub, cmd, sub
from perception import CvBridge, ImageBGR, ImageROS, light, image
from util import approx_zero, compose, lerp_signed

T = TypeVar("T")


@dataclass
class Range:
    dir: int
    dist: float


class Direction(IntEnum):
    LEFT = 1
    RIGHT = 2
    FORWARD = 3


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


State = Union[
    Wait,
    Turn,
    Drive,
    Spin,
]


@dataclass
class Model:
    cv_bridge: CvBridge
    state: State
    pose: Optional[TurtlePose]


init_model: Model = Model(
    cv_bridge=CvBridge(),
    state=Drive(),
    pose=None,
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Scan:
    ranges: List[Range]


@dataclass
class Odom:
    pose: TurtlePose


@dataclass
class Image:
    image: ImageBGR


Msg = Union[
    Scan,
    Odom,
    Image,
]


### Update ###

DRIVE_VEL_LIN: float = 0.6
DRIVE_VEL_ANG_MAX: float = 3.0

TURN_START_DIST: float = 2.0
TURN_END_DIST: float = 1.0

TURN_VEL_LIN: float = 0.60
TURN_VEL_ANG: float = 1.1

SPIN_VEL_MIN: float = 0.1 * math.pi
SPIN_VEL_MAX: float = 2.0 * math.pi

LIGHT_VAL_MIN: float = 200

RANGE_LEFT: int = 90
RANGE_RIGHT: int = 270
RANGE_OFFSET: int = 45

HALLWAY_WIDTH: float = 1.0


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(msg, Scan):
        if isinstance(model.state, Drive):
            forward = msg.ranges[0]
            left_max = longest_range(msg.ranges[45:75])
            right_max = longest_range(msg.ranges[285:315])

            choices: List[Direction] = [
                *([Direction.LEFT] if left_max.dist > TURN_START_DIST else []),
                *([Direction.RIGHT] if right_max.dist > TURN_START_DIST else []),
                *([Direction.FORWARD] if forward.dist > 0.5 else []),
            ]

            if not choices:
                return (replace(model, state=Wait()), [cmd.stop])

            direction = random.choice(choices)

            if direction == Direction.FORWARD:
                left_min = shortest_range(
                    msg.ranges[RANGE_LEFT - RANGE_OFFSET : RANGE_LEFT + RANGE_OFFSET]
                )

                right_min = shortest_range(
                    msg.ranges[RANGE_RIGHT - RANGE_OFFSET : RANGE_RIGHT + RANGE_OFFSET]
                )

                err_ang_left = (left_min.dir - RANGE_LEFT) / RANGE_OFFSET
                err_ang_right = (right_min.dir - RANGE_RIGHT) / RANGE_OFFSET

                err_ang = 0.5 * (err_ang_left + err_ang_right)
                err_lin = (left_min.dist - right_min.dist) / HALLWAY_WIDTH

                err = 0.5 * (err_lin + err_ang)
                vel_ang = lerp_signed(low=0, high=DRIVE_VEL_ANG_MAX, amount=err)

                return (model, [cmd.velocity(angular=vel_ang, linear=DRIVE_VEL_LIN)])

            return (replace(model, state=Turn(direction)), cmd.none)

        if isinstance(model.state, Turn):
            (direction, ranges_side) = (
                (1.0, msg.ranges[60:80])
                if model.state.dir == Direction.LEFT
                else (-1.0, msg.ranges[280:300])
            )

            range_front = msg.ranges[0]
            vel_ang = direction * TURN_VEL_ANG

            if ranges_under(TURN_START_DIST, ranges_side) and range_front.dist > 1.5:
                return (replace(model, state=Drive()), cmd.none)

            return (model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])

        return (model, cmd.none)

    if (
        isinstance(msg, Image)
        and isinstance(model.state, Drive)
        and model.pose is not None
    ):
        (val_max, _) = light.locate_brightest(msg.image)

        if val_max >= LIGHT_VAL_MIN:
            dir_away = model.pose.yaw + math.pi
            return (replace(model, state=Spin(dir_away)), [cmd.stop])

        return (model, cmd.none)

    if isinstance(msg, Odom):
        new_model = replace(model, pose=msg.pose)

        if isinstance(new_model.state, Spin):
            angle_off = v2.signed_angle_between(
                v2.from_angle(msg.pose.yaw),
                v2.from_angle(new_model.state.angle),
            )

            err_ang = angle_off / math.pi

            if approx_zero(err_ang, epsilon=1e-2):
                return (replace(new_model, state=Drive()), [cmd.stop])

            vel_ang = lerp_signed(SPIN_VEL_MIN, SPIN_VEL_MAX, err_ang)

            return (new_model, [cmd.turn(vel_ang)])

        return (new_model, cmd.none)

    return (model, cmd.none)


def ranges_under(dist: float, ranges: List[Range]) -> bool:
    return all([r.dist < dist for r in ranges])


def longest_range(ranges: List[Range]) -> Range:
    return max(ranges, key=lambda r: r.dist)


def shortest_range(ranges: List[Range]) -> Range:
    return min(ranges, key=lambda r: r.dist)


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    return [
        sub.laser_scan(angled_ranges),
        sub.odometry(Odom),
        sub.image_sensor(to_image_cv2(model.cv_bridge)),
    ]


def to_image_cv2(cvBridge: CvBridge) -> Callable[[ImageROS], Msg]:
    """
    Convert a ROS image message to a CV2 image in BGR format.
    """
    return compose(Image, partial(image.from_ros_image, cvBridge))


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
