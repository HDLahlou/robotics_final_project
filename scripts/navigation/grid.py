# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
import math
from typing import List

from rospy_util.mathf import sign
from rospy_util.turtle_pose import TurtlePose
from rospy_util.vector2 import Vector2
import rospy_util.vector2 as v2
from robotics_final_project.msg import Cell


@dataclass
class Grid:
    """
    A mapping of locations in 2D space to discrete cells.
    """

    len_cell: float
    origin: Vector2


def current_cell_offset(
    grid: Grid,
    cell_current: Cell,
    cell_next: Cell,
    pose: TurtlePose,
) -> float:
    """
    Find the perpendicular offset of the bot from the line
    bisecting the cell it is traveling through.
    """
    position = pose.position - grid.origin

    (length, component, direction) = (
        (cell_current.row, position.x, -sign(cell_next.col - cell_current.col))
        if cells_adjacent_in_same_row(cell_current, cell_next)
        else (cell_current.col, position.y, sign(cell_next.row - cell_current.row))
    )

    center = (length + 0.5) * grid.len_cell

    return direction * (center - component)


def next_cell_direction(
    cell_current: Cell,
    cell_next: Cell,
    pose: TurtlePose,
) -> float:
    """
    Calculate the angular displacement between the robot's current yaw and
    the direction from one cell to the next.
    """
    dir_absolute = direction_between_cells(cell_current, cell_next)

    return v2.signed_angle_between(v2.from_angle(pose.yaw), dir_absolute)


def displacement_between_cells(first: Cell, second: Cell) -> Vector2:
    """
    Calculate a vector from one cell to another.
    """
    return Vector2(second.row - first.row, second.col - first.col)


def direction_between_cells(first: Cell, second: Cell) -> Vector2:
    """
    Calculate a unit vector from one cell to another.
    """
    return v2.normalize(displacement_between_cells(first, second))


def validate_path(path: List[Cell]) -> List[Vector2]:
    """
    Check a list of cells to see if they form a traversable path.
    """
    displacements = [displacement_between_cells(*z) for z in zip(path[:-1], path[1:])]

    valid: List[Vector2] = (
        [] if any([v2.magnitude(d) != 1 for d in displacements]) else displacements
    )

    return valid


def cells_are_adjacent(first: Cell, second: Cell) -> bool:
    """
    Check if two cells are vertically or horizontally adjacent to one another.
    """
    return cells_adjacent_in_same_col(first, second) or cells_adjacent_in_same_row(
        first, second
    )


def cells_adjacent_in_same_col(first: Cell, second: Cell) -> bool:
    """
    Check if two cells are vertically (row-wise) adjacent to one another.
    """
    return abs(first.row - second.row) == 1 and (first.col - second.col == 0)


def cells_adjacent_in_same_row(first: Cell, second: Cell) -> bool:
    """
    Check if two cells are horizontally (column-wise) adjacent to one another.
    """
    return abs(first.col - second.col) == 1 and (first.row - second.row == 0)


def locate_cell(grid: Grid, cell: Cell) -> Vector2:
    """"""
    cx = (cell.row + 0.5) * grid.len_cell
    cy = (cell.col + 0.5) * grid.len_cell

    return Vector2(cx, cy) + grid.origin


def locate_pose(grid: Grid, pose: TurtlePose) -> Cell:
    """
    Map the robot's current pose to a cell in a grid.
    """
    return locate_position(grid, pose.position)


def locate_position(grid: Grid, pos: Vector2) -> Cell:
    """
    Map a 2D position vector to a cell in a grid.
    """
    (x, y) = pos - grid.origin

    row = math.floor(x / grid.len_cell)
    col = math.floor(y / grid.len_cell)

    return Cell(row, col)


def direction_to_string(dir: Vector2) -> str:
    """
    Convert a 2D vector representing a cardinal direction to a string.
    """
    if v2.equals(Vector2(1, 0), dir):
        return "south"
    if v2.equals(Vector2(0, 1), dir):
        return "east"
    if v2.equals(Vector2(-1, 0), dir):
        return "north"
    if v2.equals(Vector2(0, -1), dir):
        return "west"

    return "invalid"