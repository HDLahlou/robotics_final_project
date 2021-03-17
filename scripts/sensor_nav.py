#!/usr/bin/env python3

"""
Sensory navigation.
"""

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import Any, Callable, List, Optional, Tuple, Union

import numpy as np

import rospy
from rospy_util.turtle_pose import TurtlePose
import rospy_util.vector2 as v2
import scipy.ndimage as ndi
from sensor_msgs.msg import LaserScan

from controller import Cmd, Controller, Sub, cmd, sub
from perception import CvBridge, ImageBGR, ImageROS, image, light
from util import approx_zero, compose, head, lerp_signed


@dataclass
class Range:
    """
    A single LiDAR range.
    """

    dir: int
    dist: float


class Direction(IntEnum):
    """
    Relative directions for maze navigation.
    """

    LEFT = 1
    RIGHT = 2
    FORWARD = 3


### Model ###


@dataclass
class Stop:
    """
    Stop moving.
    """

    reason: str


@dataclass
class Drive:
    """
    Drive forward.
    """


@dataclass
class Turn:
    """
    Turn in the given direction.
    """

    dir: Direction


@dataclass
class Cross:
    pass


@dataclass
class Spin:
    """
    Spin to face the given absolute angle.
    """

    angle: float


# Possible states of the robot

State = Union[
    Stop,
    Turn,
    Cross,
    Drive,
    Spin,
]


@dataclass
class Model:
    """
    Model of the robot.

    @attribute `cv_bridge`: CV2 bridge for image format conversion.

    @attribute `state`: Current state of the robot.

    @attribute `pose`: Last measured pose of the robot.
    """

    cv_bridge: CvBridge
    directions: List[Direction]
    state: State
    pose: Optional[TurtlePose]


init_model: Model = Model(
    cv_bridge=CvBridge(),
    directions=[
        Direction.RIGHT,
        Direction.LEFT,
        Direction.RIGHT,
        Direction.FORWARD,
        Direction.RIGHT,
    ],
    state=Drive(),
    pose=None,
)

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Scan:
    """
    Scan ranges as measured by robot LiDAR.
    """

    ranges: List[Range]


@dataclass
class Odom:
    """
    Pose of the robot as estimated by odometry.
    """

    pose: TurtlePose


@dataclass
class Image:
    """
    Image taken by the robot camera.
    """

    image: ImageBGR


# Inbound messages

Msg = Union[
    Scan,
    Odom,
    Image,
]


### Update ###

# Driving movement

DRIVE_VEL_LIN: float = 0.6
DRIVE_VEL_ANG_MAX: float = 3.0

# Driving conditions

DRIVE_WALL_AHEAD_DIST: float = 0.5

# Turning movement

TURN_VEL_LIN: float = 0.60
TURN_VEL_ANG: float = 1.1

# Turning conditions

TURN_WALL_AHEAD_DIST: float = 1.5
TURN_START_DIST: float = 2.0
TURN_END_DIST: float = 2.0

# Spinning movement

SPIN_VEL_MIN: float = 0.1 * math.pi
SPIN_VEL_MAX: float = 2.0 * math.pi

# Light detection threshold

LIGHT_VAL_MIN: float = 200

# LiDAR range directions

RANGE_LEFT: int = 90
RANGE_RIGHT: int = 270
RANGE_OFFSET: int = 45

# Maze hallway width

HALLWAY_WIDTH: float = 1.0

# Threshold for detecting sharp range jumps

JUMP_THRESH: float = 0.5
INFLECTION_THRESH: float = 0.01


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    """
    Given an inbound message and the current robot model, produce a new model
    and a list of commands to execute.
    """
    if isinstance(model.state, Stop):
        return (model, [cmd.stop])

    if (direction := head(model.directions)) is None:
        return (replace(model, state=Stop("out of directions")), cmd.none)

    if isinstance(msg, Scan):
        # Received a message from robot LiDAR.

        print(critical_ranges(msg.ranges))
        return (model, cmd.none)

        if isinstance(model.state, Drive):
            # Compute movement choices.

            forward = msg.ranges[0]
            left_max = longest_range(msg.ranges[45:75])
            right_max = longest_range(msg.ranges[285:315])

            choices: List[Direction] = [
                *([Direction.LEFT] if left_max.dist > TURN_START_DIST else []),
                *([Direction.RIGHT] if right_max.dist > TURN_START_DIST else []),
                *([Direction.FORWARD] if forward.dist > DRIVE_WALL_AHEAD_DIST else []),
            ]

            if not choices:
                return (transition(model, state=Stop("dead end")), cmd.none)

            if choices == [Direction.FORWARD]:
                return drive_forward(msg.ranges, model)

            if direction not in choices:
                return (
                    transition(model, state=Stop("impossible direaction")),
                    cmd.none,
                )

            if direction == Direction.FORWARD:
                return (transition(model, state=Cross()), cmd.none)

            # Turning left or right; transition to turn state.

            return (transition(model, state=Turn(direction)), cmd.none)

        if isinstance(model.state, Cross):
            ranges_left = msg.ranges[80:100]
            ranges_right = msg.ranges[260:280]

            if ranges_under(0.8, ranges_left) and ranges_under(0.8, ranges_right):
                new_model = replace(model, directions=model.directions[1:])
                return (transition(new_model, state=Drive()), cmd.none)

            return (model, [cmd.drive(0.6)])  # drive_forward(msg.ranges, model)

        if isinstance(model.state, Turn):
            # Choose sign of velocity and portion of LiDAR ranges for specified
            # turning direction.

            (direction, ranges_side) = (
                (1.0, msg.ranges[60:80])
                if model.state.dir == Direction.LEFT
                else (-1.0, msg.ranges[280:300])
            )

            range_front = msg.ranges[0]

            if (
                ranges_under(TURN_END_DIST, ranges_side)
                and range_front.dist > TURN_WALL_AHEAD_DIST
            ):
                # Wall is close to side of bot, no obstructions ahead; assume turn is over
                # and transition to driving state.
                new_model = replace(model, directions=model.directions[1:])
                return (transition(new_model, state=Drive()), cmd.none)

            # Perform turn with computed angular velocity.

            vel_ang = direction * TURN_VEL_ANG

            return (model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])

        return (model, cmd.none)

    if (
        isinstance(msg, Image)
        and isinstance(model.state, Drive)
        and model.pose is not None
    ):
        # Compute value of brightest area in image.

        (val_max, _) = light.locate_brightest(msg.image)

        if val_max >= LIGHT_VAL_MIN:
            # Value is sufficiently high; consider the detected object a light
            # and begin spinning around to escape.

            dir_away = model.pose.yaw + math.pi
            return (transition(model, state=Spin(dir_away)), [cmd.stop])

        return (model, cmd.none)

    if isinstance(msg, Odom):
        # Update model with new robot pose.

        new_model = replace(model, pose=msg.pose)

        if isinstance(new_model.state, Spin):
            # Compute angle between the robot and the final direction of the spin
            # maneuver.

            angle_off = v2.signed_angle_between(
                v2.from_angle(msg.pose.yaw),
                v2.from_angle(new_model.state.angle),
            )

            err_ang = angle_off / math.pi

            if approx_zero(err_ang, epsilon=1e-2):
                # Angular error sufficiently low; consider spin complete and
                # switch to driving state.

                return (transition(new_model, state=Drive()), [cmd.stop])

            # Compute angular velocity and perform spin.

            vel_ang = lerp_signed(SPIN_VEL_MIN, SPIN_VEL_MAX, err_ang)

            return (new_model, [cmd.turn(vel_ang)])

        return (new_model, cmd.none)

    # None of the above cases match; no update.

    return (model, cmd.none)


def transition(model: Model, state: State) -> Model:
    print(f"Begin {state}")
    return replace(model, state=state)


def drive_forward(ranges: List[Range], model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    left_min = shortest_range(
        ranges[RANGE_LEFT - RANGE_OFFSET : RANGE_LEFT + RANGE_OFFSET]
    )

    right_min = shortest_range(
        ranges[RANGE_RIGHT - RANGE_OFFSET : RANGE_RIGHT + RANGE_OFFSET]
    )

    # Computer angular and positional errors for current orientation.

    err_ang_left = (left_min.dir - RANGE_LEFT) / RANGE_OFFSET
    err_ang_right = (right_min.dir - RANGE_RIGHT) / RANGE_OFFSET

    err_ang = 0.5 * (err_ang_left + err_ang_right)
    err_lin = (left_min.dist - right_min.dist) / HALLWAY_WIDTH

    # Compute aggergate error and corrective angular velocity.

    err = 0.5 * (err_lin + err_ang)
    vel_ang = lerp_signed(low=0, high=DRIVE_VEL_ANG_MAX, amount=err)

    # Turn with corrective angular velocity while moving forward.

    return (model, [cmd.velocity(angular=vel_ang, linear=DRIVE_VEL_LIN)])


def ranges_under(dist: float, ranges: List[Range]) -> bool:
    """
    Check if all LiDAR ranges in the given list are shorter than the specified
    distance.
    """
    return all([r.dist < dist for r in ranges])


def longest_range(ranges: List[Range]) -> Range:
    """
    Get the longest LiDAR range from the given list.
    WARNING: Partial function, will explode if given an empty list.
    """
    return max(ranges, key=lambda r: r.dist)


def shortest_range(ranges: List[Range]) -> Range:
    """
    Get the shortest LiDAR range from the given list.
    WARNING: Partial function, will explode if given an empty list.
    """
    return min(ranges, key=lambda r: r.dist)


def sharp_jump(ranges: List[Range], i: int) -> bool:
    """
    Check if a LiDAR range is a sharp jump from the previous value.
    """
    r = ranges[i].dist
    s = ranges[(i + 1) % 360].dist

    return abs(r - s) > JUMP_THRESH and np.isfinite(r) and np.isfinite(s)


def inflection_point(ranges: List[Range], i: int) -> bool:
    """
    Check if a LiDAR range is an inflection point.
    """
    r = ranges[i]
    q = ranges[i - 1]
    s = ranges[(i + 1) % 360]

    rv = v2.scale(v2.from_angle(math.radians(r.dir)), r.dist)
    qv = v2.scale(v2.from_angle(math.radians(q.dir)), q.dist)
    sv = v2.scale(v2.from_angle(math.radians(s.dir)), s.dist)

    qr_mag = v2.magnitude(rv - qv)
    rs_mag = v2.magnitude(sv - rv)
    sq_mag = v2.magnitude(qv - sv)

    semi_p = (qr_mag + rs_mag + sq_mag) / 2

    triangle_area = math.sqrt(
        semi_p * (semi_p - qr_mag) * (semi_p - rs_mag) * (semi_p - sq_mag)
    )

    q_residual = 2 * triangle_area / rs_mag
    s_residual = 2 * triangle_area / qr_mag

    print(f"angle: {i}; q: {q_residual}; s: {s_residual}")
    print(f"r_dist: {r.dist}; q_dist: {q.dist}; s_dist: {s.dist}")

    return (
        # ((r.dist > q.dist and r.dist > s.dist) or (r.dist < q.dist and r.dist < s.dist))
        (q_residual > INFLECTION_THRESH)
        and (s_residual > INFLECTION_THRESH)
    )


def critical_ranges(ranges: List[Range]) -> List[Range]:
    """
    Get a list of LiDAR ranges where sharp jumps or inflection points occur.
    """

    # smooth_distances = ndi.gaussian_filter(
    #     [r.dist for r in ranges],
    #     sigma=1.0,
    # )

    # smooth_ranges = [Range(i, d) for i, d in enumerate(smooth_distances)]

    return [
        r for r in ranges if inflection_point(ranges, r.dir)  # or sharp_jump(ranges, i)
    ]


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    """
    Subscriptions from which to receive messages.
    """
    return [
        sub.image_sensor(to_image_cv2(model.cv_bridge)),
        sub.laser_scan(lidar_ranges),
        sub.odometry(Odom),
    ]


def to_image_cv2(cvBridge: CvBridge) -> Callable[[ImageROS], Msg]:
    """
    Convert a ROS image message to a CV2 image in BGR format.
    """
    return compose(Image, partial(image.from_ros_image, cvBridge))


def lidar_ranges(scan: LaserScan) -> Msg:
    """
    Convert raw laserscan data to angled LiDAR ranges.
    """
    return Scan([Range(deg, dist) for (deg, dist) in enumerate(scan.ranges)])


### Run ###


def run() -> None:
    rospy.init_node("laser_sensor_nav")

    # Run the ROS controller

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
