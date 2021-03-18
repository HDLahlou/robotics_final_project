# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
import math

from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2


@dataclass
class Grid:
    len_cell: float
    origin: Vector2


@dataclass
class Cell:
    row: int
    col: int


def next_cell_direction(
    grid: Grid,
    pose: TurtlePose,
    next_cell: Cell,
) -> float:
    current_cell = locate_pose(grid, pose)
    dir_absolute = direction_between_cells(current_cell, next_cell)

    return v2.signed_angle_between(v2.from_angle(pose.yaw), dir_absolute)


def direction_between_cells(first: Cell, second: Cell) -> Vector2:
    displacement = Vector2(second.col - first.col, second.row - first.row)
    return v2.normalize(displacement)


def cells_are_adjacent(first: Cell, second: Cell) -> bool:
    # TODO - could rewrite in terms of above function
    displacement = (second.row - first.row, second.col - first.col)
    return displacement in [(0, 1), (1, 0), (-1, 0), (0, -1)]


def locate_pose(grid: Grid, pose: TurtlePose) -> Cell:
    return locate_position(grid, pose.position)


def locate_position(grid: Grid, pos: Vector2) -> Cell:
    (x, y) = pos - grid.origin

    row = math.floor(y / grid.len_cell)
    col = math.floor(x / grid.len_cell)

    return Cell(row, col)
