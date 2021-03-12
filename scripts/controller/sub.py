"""
ROS controller subscriptions.
"""

# pyright: reportMissingTypeStubs=false

from typing import Callable, TypeVar

from nav_msgs.msg import Odometry
from rospy_util.controller import Sub
from rospy_util.turtle_pose import TurtlePose
import rospy_util.turtle_pose as tp
from sensor_msgs.msg import Image, LaserScan

__all__ = (
    "arm_result",
    "image_sensor",
    "laser_scan",
    "odometry",
    "optimal_actions",
    "robot_action",
)

Msg = TypeVar("Msg")


def odometry(to_msg: Callable[[TurtlePose], Msg]) -> Sub[Odometry, Msg]:
    """
    Receive odometry sensor data.
    """
    return Sub(
        topic_name="/tb1/odom",
        message_type=Odometry,
        to_msg=lambda odom: to_msg(tp.from_pose(odom.pose.pose)),
    )


def laser_scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[LaserScan, Msg]:
    """
    Receive LiDAR sensor data.
    """
    return Sub(
        topic_name="/tb1/scan",
        message_type=LaserScan,
        to_msg=to_msg,
    )


def image_sensor(to_msg: Callable[[Image], Msg]) -> Sub[Image, Msg]:
    """
    Receive image sensor data.
    """
    return Sub(
        topic_name="/tb1/camera/rgb/image_raw",
        message_type=Image,
        to_msg=to_msg,
    )
