"""
General utilities.
"""

# pyright: reportMissingTypeStubs=false

from typing import Callable, List, Optional, TypeVar

import rospy_util.mathf as mathf

__all__ = (
    "approx_zero",
    "compose",
    "head",
    "set_under_abs",
)

T = TypeVar("T")
U = TypeVar("U")
V = TypeVar("V")


def set_under_abs(value: float, low: float) -> float:
    """
    Take the absolute minimum of two numbers, preserving sign.
    """
    return mathf.sign(value) * min(abs(value), abs(low))


def approx_zero(value: float, epsilon: float = 0.05) -> bool:
    """
    Check if a number is approximately zero.
    """
    return abs(value) < epsilon


def head(xs: List[T]) -> Optional[T]:
    """
    Try to get the first element in a list.
    """
    return None if not xs else xs[0]


def compose(f: Callable[[U], V], g: Callable[[T], U]) -> Callable[[T], V]:
    """
    Compose two functions.
    """
    return lambda x: f(g(x))
