# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
import math

from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2
from robotics_final_project.msg import Cell


@dataclass
class Grid:
    len_cell: float
    origin: Vector2


def current_cell_offset(grid: Grid, pose: TurtlePose) -> float:
    current_cell = locate_pose(grid, pose)
    position = pose.position - grid.origin

    (length, component) = (
        (current_cell.row, position.y)
        if facing_horizontal(pose.yaw)
        else (current_cell.col, position.x)
    )

    center = (length + 0.5) * grid.len_cell

    return center - component


def facing_horizontal(yaw: float) -> bool:
    # TODO - eh (should encode directions manually) [also looks gross]
    return (
        math.pi / -4.0 < yaw < math.pi / 4.0 or math.pi / -0.75 > yaw > math.pi / 0.75
    )


def next_cell_direction(
    grid: Grid,
    pose: TurtlePose,
    next_cell: Cell,
) -> float:
    current_cell = locate_pose(grid, pose)
    dir_absolute = direction_between_cells(current_cell, next_cell)

    return v2.signed_angle_between(v2.from_angle(pose.yaw), dir_absolute)


def next_cell_adjacent(
    grid: Grid,
    pose: TurtlePose,
    next_cell: Cell,
) -> bool:
    current_cell = locate_pose(grid, pose)
    return cells_are_adjacent(current_cell, next_cell)


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
