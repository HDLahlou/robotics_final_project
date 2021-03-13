"""
Perception of light.
"""

# pyright: reportMissingTypeStubs=false

from typing import Tuple

import cv2

from perception.image import ImageBGR


def locate_brightest(img: ImageBGR) -> Tuple[float, Tuple[int, int]]:
    grayscale = cv2.cvtColor(img.data, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(grayscale, (41, 41), 0)

    (_, val_max, _, loc_max) = cv2.minMaxLoc(blurred)

    return (val_max, loc_max)
