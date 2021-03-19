"""
Perception of light.
"""

# pyright: reportMissingTypeStubs=false

from typing import Tuple

import cv2

from perception.image import ImageBGR

GAUSSIAN_RADIUS: int = 41


def locate_brightest(img: ImageBGR) -> Tuple[float, Tuple[int, int]]:
    """
    Return the value and pixel coordinates of the brightest area in the given
    image.

    This function uses `cv2.minMaxLoc`, which searches for the brightest single
    pixel in the input image. To search a greater area, the image is
    pre-processed with a gaussian blur filter, averaging pixel brightness and
    hopefully removing any noise that could otherwise be interpreted as the
    brightest pixels in the image.

    See: https://www.pyimagesearch.com/2014/09/29/finding-brightest-spot-image-using-python-opencv/
    """
    grayscale = cv2.cvtColor(img.data, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(grayscale, (GAUSSIAN_RADIUS, GAUSSIAN_RADIUS), 0)

    (_, val_max, _, loc_max) = cv2.minMaxLoc(blurred)

    return (val_max, loc_max)
