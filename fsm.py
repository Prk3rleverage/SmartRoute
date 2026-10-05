from enum import Enum


class RobotState(Enum):
    FOLLOW_PATH = 1
    AVOID_OBSTACLE = 2
    RETURN_TO_PATH = 3
    GOAL_REACHED = 4
