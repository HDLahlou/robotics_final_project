#!/usr/bin/env python3

"""
Sensory navigation.
"""

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass, replace
from enum import IntEnum
from functools import partial
import math
from typing import (
    Any,
    Callable,
    Generic,
    Iterator,
    List,
    Optional,
    Tuple,
    TypeVar,
    Union,
)

import numpy as np

import rospy
from rospy_util.turtle_pose import TurtlePose
import rospy_util.vector2 as v2
from sensor_msgs.msg import LaserScan

from controller import Cmd, Controller, Sub, cmd, sub
from perception import CvBridge, ImageBGR, ImageROS, image, light
from util import approx_zero, compose, head, lerp_signed

T = TypeVar("T")


@dataclass
class Range:
    """
    A single LiDAR range.
    """

    dir: int
    dist: float


@dataclass
class Corners(Generic[T]):
    top_left: T
    bot_left: T
    top_right: T
    bot_right: T

    def __iter__(self) -> Iterator[T]:
        return iter([self.top_left, self.bot_left, self.bot_right, self.top_right])

    def __setitem__(self, key: int, item: T) -> None:
        if key == 0:
            self.top_left = item
        elif key == 1:
            self.bot_left = item
        elif key == 2:
            self.bot_right = item
        elif key == 3:
            self.top_right = item

    def __getitem__(self, key: int) -> T:
        if key == 0:
            return self.top_left
        if key == 1:
            return self.bot_left
        if key == 2:
            return self.bot_right
        if key == 3:
            return self.top_right

        raise IndexError("bad corner index")


CornerRanges = Corners[Optional[Range]]


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
    corners: CornerRanges


@dataclass
class Cross:
    corners: CornerRanges


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
TURN_VEL_ANG: float = 3.0

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
CORNER_THRESH_DIST: float = 0.5
CORNER_THRESH_ANGLE: float = 15
CRITICAL_RANGE_LIMIT: float = HALLWAY_WIDTH * 1.8
CRITICAL_RANGE_ANGLE_DIFF: float = 25

CORNER_ANGLES_ENTRY: List[int] = [15, 70, 290, 345]
CORNER_ANGLE_OFFSET: int = 15


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

        # print(critical_ranges(msg.ranges))
        # return (model, cmd.none)

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

            corners = track_corners(msg.ranges)

            if direction == Direction.FORWARD:
                return (transition(model, state=Cross(corners)), cmd.none)

            # Turning left or right; transition to turn state.

            return (transition(model, state=Turn(direction, corners)), cmd.none)

        if isinstance(model.state, Cross):
            ranges_left = msg.ranges[80:100]
            ranges_right = msg.ranges[260:280]

            if ranges_under(0.8, ranges_left) and ranges_under(0.8, ranges_right):
                new_model = replace(model, directions=model.directions[1:])
                return (transition(new_model, state=Drive()), cmd.none)

            # drive_forward(msg.ranges, model)
            err = corner_err(Direction.FORWARD, model.state.corners)
            vel_ang = err * TURN_VEL_ANG

            return (model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])

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

            corners = update_corners(model.state.corners, msg.ranges)

            if not any(corners):
                return (model, cmd.none)

            err = corner_err(model.state.dir, corners)

            print(f"corner_err: {err}")

            # Perform turn with computed angular velocity.

            vel_ang = err * TURN_VEL_ANG
            # vel_ang = direction * TURN_VEL_ANG

            new_model = replace(model, state=Turn(model.state.dir, corners))

            return (new_model, [cmd.velocity(angular=vel_ang, linear=TURN_VEL_LIN)])

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


def corner_err(dir: Direction, corners: CornerRanges) -> float:
    """
    Calculate a proportional control value for a turn maneuver based on corner
    locations.
    """

    print("corners: ", corners)

    if dir == Direction.LEFT and corners.bot_left is not None:
        if corners.top_left is not None:
            return opp_side_err(corners.bot_left, corners.top_left)
        elif corners.bot_right is not None:
            return adj_side_err(corners.bot_left, corners.bot_right)
        elif corners.top_right is not None:
            return dia_side_err(corners.bot_left, corners.top_right)

    elif dir == Direction.RIGHT and corners.bot_right is not None:
        if corners.top_right is not None:
            return opp_side_err(corners.bot_right, corners.top_right)
        elif corners.bot_left is not None:
            return adj_side_err(corners.bot_right, corners.bot_left)
        elif corners.top_left is not None:
            return dia_side_err(corners.bot_right, corners.top_left)

    else:
        if corners.top_left is not None and corners.top_right is not None:
            return opp_side_err(corners.top_left, corners.top_right)
        elif corners.top_left is not None and corners.bot_left is not None:
            return adj_side_err(corners.top_left, corners.bot_left)
        elif corners.top_right is not None and corners.bot_right is not None:
            return adj_side_err(corners.top_right, corners.bot_right)

    return 0


def opp_side_err(pivot_r: Range, opp_r: Range) -> float:
    """
    Calculate an angular displacement between the bot's yaw
    and the center of a junction exit based on the two corners on either side
    of it.
    """
    pivot_v = v2_from_range(pivot_r)
    opp_v = v2_from_range(opp_r)
    return v2.signed_angle_between(pivot_v + opp_v, v2.right) / (math.pi / 2)


def adj_side_err(pivot_r: Range, adj_r: Range) -> float:
    """
    Calculate an angular displacement between the bot's yaw
    and the center of a junction exit based on the two corners on either
    side of an adjacent exit.
    """
    pivot_v = v2_from_range(pivot_r)
    adj_v = v2_from_range(adj_r)
    side_v = pivot_v - adj_v
    if -180 < pivot_r.dir - adj_r.dir < 180:
        dest_v = pivot_v + v2.scale(v2.rotate(side_v, math.pi / 2), 0.5)
    else:
        dest_v = pivot_v + v2.scale(v2.rotate(side_v, -math.pi / 2), 0.5)
    return v2.signed_angle_between(dest_v, v2.right) / (math.pi / 2)


def v2_from_range(r: Range) -> v2.Vector2:
    return v2.scale(v2.from_angle(math.radians(r.dir)), r.dist)


def dia_side_err(pivot_r: Range, dia_r: Range) -> float:
    """
    Calculate an angular displacement between the bot's yaw
    and the center of a junction exit based on two corners opposite
    each other.
    """
    pivot_v = v2_from_range(pivot_r)
    dia_v = v2_from_range(dia_r)
    mid_v = v2.scale(pivot_v + dia_v, 0.5)
    if pivot_r.dir > 180:
        dest_v = mid_v + v2.scale(
            v2.rotate(pivot_v - mid_v, math.pi / 4), 1 / math.sqrt(2)
        )
    else:
        dest_v = mid_v + v2.scale(
            v2.rotate(pivot_v - mid_v, -math.pi / 4), 1 / math.sqrt(2)
        )
    return v2.signed_angle_between(v2.right, dest_v) / (math.pi / 2)


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
    q = ranges[i - 1].dist
    s = ranges[(i + 1) % 360].dist

    return (r - s) > JUMP_THRESH or (r - q) > JUMP_THRESH


def corner(ranges: List[Range], i: int) -> bool:
    """
    Check if a LiDAR range is a corner.
    """
    r = ranges[i]
    q = ranges[i - 5]
    s = ranges[(i + 5) % 360]

    rv = v2_from_range(r)
    qv = v2_from_range(q)
    sv = v2_from_range(s)

    qs_dist = v2.magnitude(sv - qv)

    central_angle = math.degrees(v2.signed_angle_between(qv - rv, sv - rv))

    return (abs(abs(central_angle) - 90) < CORNER_THRESH_ANGLE) and (
        qs_dist < CORNER_THRESH_DIST
    )


def critical_ranges(ranges: List[Range]) -> List[Range]:
    """
    Get a list of LiDAR ranges where sharp jumps or corners occur.
    """

    potential_ranges = [
        r
        for r in ranges
        if r.dist < CRITICAL_RANGE_LIMIT
        and (sharp_jump(ranges, r.dir) or corner(ranges, r.dir))
    ]

    return [
        r
        for i, r in enumerate(potential_ranges)
        if (potential_ranges[i].dir - potential_ranges[i - 1].dir) % 360
        > CRITICAL_RANGE_ANGLE_DIFF
    ]


def track_corners(ranges: List[Range]) -> CornerRanges:
    """
    Create a list of corner ranges for use when passing through a junction.
    When entering the junction, each corner is mapped to a list element:
    i = 0: top left
    i = 1: bottom left
    i = 2: bottom right
    i = 3: top right
    """

    crit_ranges = critical_ranges(ranges)
    corners = CornerRanges(None, None, None, None)

    forward_ranges = [r for r in crit_ranges if 150 > r.dir or r.dir > 210]

    for r in forward_ranges:
        min_corner = min(range(4), key=lambda x: abs(CORNER_ANGLES_ENTRY[x] - r.dir))
        if corners[min_corner] is None:
            corners[min_corner] = r

    print("Tracking corners: ", corners)
    return corners


def update_corners(corners: CornerRanges, ranges: List[Range]) -> CornerRanges:
    """
    Update a list of corners during a turn.
    """

    crit_ranges = critical_ranges(ranges)

    if not crit_ranges:
        return corners
    print("crit_ranges: ", crit_ranges)
    new_corners = CornerRanges(None, None, None, None)

    for i in range(4):
        if (corner := corners[i]) is not None:
            min_range = min(crit_ranges, key=lambda r: abs(r.dir - corner.dir))
            if not min_range in new_corners:
                new_corners[i] = min_range

    return new_corners


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
