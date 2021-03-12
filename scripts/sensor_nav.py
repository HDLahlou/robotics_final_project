#!/usr/bin/env python3

# pyright: reportMissingTypeStubs=false

from dataclasses import dataclass
from typing import Any, List, Tuple, Union

import rospy

from controller import Cmd, Controller, Sub, cmd, sub

### Model ###


@dataclass
class Wait:
    pass


@dataclass
class Drive:
    pass


Model = Union[
    Wait,
    Drive,
]


init_model: Model = Wait()

init: Tuple[Model, List[Cmd[Any]]] = (init_model, cmd.none)


### Messages ###


@dataclass
class Scan:
    ranges: List[float]


Msg = Union[Scan]


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, List[Cmd[Any]]]:
    print(msg.ranges)

    return (model, cmd.none)


### Subscriptions ###


def subscriptions(model: Model) -> List[Sub[Any, Msg]]:
    return [sub.laser_scan(lambda s: Scan(s.ranges))]


### Run ###


def run() -> None:
    rospy.init_node("q_learning_robot_action")

    # Run the ROS controller

    Controller.run(
        init=init,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
