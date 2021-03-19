"""
Images of various formats.
"""

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Any, Generic, Literal, TypeVar

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageROS


T = TypeVar("T")


@dataclass
class Image(Generic[T]):
    """
    An image of a known format.
    """

    data: Any
    height: int
    width: int


ImageBGR = Image[Literal["BGR"]]


def from_ros_image(bridge: CvBridge, img: ImageROS) -> ImageBGR:
    """
    Create a BGR image from a ROS image message.
    """

    data = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

    return ImageBGR(
        data=data,
        height=img.height,
        width=img.width,
    )
