#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, TypeVar, Union

import rospy
import rospy_util.mathf as mathf
from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2
from sensor_msgs.msg import Image, LaserScan

from controller import Cmd, Controller, Sub, cmd, sub
from perception import CvBridge, ImageBGR, ImageROS, light, image
from util import compose

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
DRIVE_KP_ANG: float = 3.0

TURN_START_DIST: float = 2.0
TURN_END_DIST: float = 1.0

TURN_VEL_LIN: float = 0.65
TURN_KP_ANG: float = 1.1

SPIN_VEL_MIN: float = 0.1 * math.pi
SPIN_VEL_MAX: float = 2.0 * math.pi

LIGHT_VAL_MIN: float = 200


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    if isinstance(msg, Scan):
        if isinstance(model.state, Drive):
            left_max = max(msg.ranges[45:75], key=lambda r: r.dist)

            if left_max.dist > TURN_START_DIST:
                print("turn left")
                return (replace(model, state=Turn(Direction.LEFT)), cmd.none)

            right_max = max(msg.ranges[285:315], key=lambda r: r.dist)

            if right_max.dist > TURN_START_DIST:
                print("turn right")
                return (replace(model, state=Turn(Direction.RIGHT)), cmd.none)

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

        if isinstance(model.state, Turn):
            vel_ang = (1.0 if model.state.dir == Direction.LEFT else -1.0) * TURN_KP_ANG
            ranges_side = (
                msg.ranges[60:80]
                if model.state.dir == Direction.LEFT
                else msg.ranges[280:300]
            )
            range_front = msg.ranges[0]

            if (
                all([r.dist < TURN_END_DIST for r in ranges_side])
                and range_front.dist > 1.5
            ):
                print("drive")
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
            return (replace(model, state=Spin(dir_away)), cmd.none)

        return (model, cmd.none)

    if isinstance(msg, Odom):
        new_model = replace(model, pose=msg.pose)

        if isinstance(new_model.state, Spin):
            angle_off = v2.signed_angle_between(
                v2.from_angle(msg.pose.yaw),
                v2.from_angle(new_model.state.angle),
            )

            err_ang = angle_off / math.pi

            print("err_ang: ", err_ang)

            if abs(err_ang) < 1e-2:
                return (replace(new_model, state=Drive()), cmd.none)

            vel_ang = mathf.lerp(SPIN_VEL_MIN, SPIN_VEL_MAX, err_ang)

            return (new_model, [cmd.turn(vel_ang)])

        return (new_model, cmd.none)

    return (model, cmd.none)


def delta_angle(pose: TurtlePose, target: Vector2) -> float:
    """
    Compute the angle between the direction of the TurtleBot and the target direction.
    """
    direction_facing = v2.from_angle(pose.yaw)
    return v2.signed_angle_between(target, direction_facing)


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
