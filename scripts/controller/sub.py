"""
ROS controller subscriptions.
"""

# pyright: reportMissingTypeStubs=false

from typing import Any, Callable, List, TypeVar

from nav_msgs.msg import Odometry
from rospy_util.controller import Sub
from rospy_util.turtle_pose import TurtlePose
import rospy_util.turtle_pose as tp
from sensor_msgs.msg import Image, LaserScan

from robotics_final_project.msg import Cell, Path

__all__ = (
    "image_sensor",
    "laser_scan",
    "none",
    "odometry",
)

Msg = TypeVar("Msg")


def directions(to_msg: Callable[[List[Cell]], Msg]) -> Sub[Path, Msg]:
    """
    Receive directions for a path to follow.
    """
    return Sub(
        topic_name="/directions",
        message_type=Path,
        to_msg=lambda p: to_msg(p.path),
    )


def odometry(bot: str, to_msg: Callable[[TurtlePose], Msg]) -> Sub[Odometry, Msg]:
    """
    Receive odometry sensor data.
    """
    return Sub(
        topic_name=f"/{bot}/odom",
        message_type=Odometry,
        to_msg=lambda odom: to_msg(tp.from_pose(odom.pose.pose)),
    )


def laser_scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[LaserScan, Msg]:
    """
    Receive LiDAR sensor data.
    """
    return Sub(
        topic_name="/hunter/scan",
        message_type=LaserScan,
        to_msg=to_msg,
    )


def image_sensor(to_msg: Callable[[Image], Msg]) -> Sub[Image, Msg]:
    """
    Receive image sensor data.
    """
    return Sub(
        topic_name="/hunter/camera/rgb/image_raw",
        message_type=Image,
        to_msg=to_msg,
    )


none: List[Sub[Any, Any]] = []
