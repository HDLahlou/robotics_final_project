"""
ROS controller commands.
"""

# pyright: reportMissingTypeStubs=false


from geometry_msgs.msg import Twist, Vector3
from rospy_util.controller.cmd import Cmd, none

__all__ = (
    "drive",
    "none",
    "stop",
    "turn",
    "velocity",
)


def turn(vel_angular: float) -> Cmd[Twist]:
    """
    Instruct the robot to turn with the given angular velocity.
    """
    return velocity(linear=0.0, angular=vel_angular)


def drive(vel_linear: float) -> Cmd[Twist]:
    """
    Instruct the robot to drive forward with the given linear velocity.
    """
    return velocity(linear=vel_linear, angular=0.0)


def velocity(linear: float, angular: float) -> Cmd[Twist]:
    """
    Instruct the robot to move with the given velocities.
    """
    return Cmd(
        topic_name="/cmd_vel",
        message_type=Twist,
        message_value=twist_from_velocities(linear, angular),
    )


def twist_from_velocities(linear_x: float, angular_z: float) -> Twist:
    """
    Create a twist message from the given velocities.
    """
    return Twist(
        angular=Vector3(x=0.0, y=0.0, z=angular_z),
        linear=Vector3(x=linear_x, y=0.0, z=0.0),
    )


# Instruct the robot to stop moving.

stop: Cmd[Twist] = velocity(linear=0.0, angular=0.0)
