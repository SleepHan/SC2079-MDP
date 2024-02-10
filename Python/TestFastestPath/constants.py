from enum import IntEnum
from queue import Queue

arduino_queue = Queue(10)
general_queue = Queue(10)


class Bearing(IntEnum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6

    NORTH_EAST = 1
    SOUTH_EAST = 3
    SOUTH_WEST = 5
    NORTH_WEST = 7

    @staticmethod
    def next_bearing(current_bearing):
        return Bearing((current_bearing.value + 2) % 8)

    @staticmethod
    def prev_bearing(current_bearing):
        return Bearing((current_bearing.value + 6) % 8)

    @staticmethod
    def next_bearing_diag(current_bearing):
        return Bearing((current_bearing.value + 1) % 8)

    @staticmethod
    def prev_bearing_diag(current_bearing):
        return Bearing((current_bearing.value + 7) % 8)

    @staticmethod
    def next_bearing_diag(current_bearing):
        return Bearing((current_bearing.value + 1) % 8)

    @staticmethod
    def prev_bearing_diag(current_bearing):
        return Bearing((current_bearing.value + 7) % 8)

    @staticmethod
    def is_diag_bearing(current_bearing):
        return current_bearing.value % 2 == 1


class MOVEMENT(IntEnum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2

    FORWARD_DIAG = 3
    LEFT_DIAG = 4
    RIGHT_DIAG = 5


class COST(IntEnum):
    INFINITE_COST = 9999
    MOVE_COST = 10
    MOVE_COST_DIAG = 15
    TURN_COST = 20
    TURN_COST_DIAG = 10
    WAYPONT_PENALTY = 1000