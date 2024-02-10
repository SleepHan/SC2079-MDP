import logging

import config
import numpy as np
from constants import Bearing

# ----------------------------------------------------------------------
#   Map Legend:
#   0 - not explored
#   1 - explored
# ----------------------------------------------------------------------

map_is_explored = \
    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

# ----------------------------------------------------------------------
#   Map Legend:
#   0 - free
#   1 - obstacle
# ----------------------------------------------------------------------
map_sim = \
    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
     [1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

# ----------------------------------------------------------------------
#   Map Legend:
#   0 - free
#   1 - obstacle
#   2 - virtual wall
# ----------------------------------------------------------------------

# map_virtual = \
#     [[0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#      [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
#      [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

map_virtual = \
    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

# ----------------------------------------------------------------------
#   Map Legend:
#   -ve - free
#   +ve - obstacle
# ----------------------------------------------------------------------

map_virtual_w = \
    [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]


class Map:
    # def __init__(self):

    def is_explored(self, x, y):
        try:
            return map_is_explored[y][x]
        except IndexError:
            logging.debug(str(y) + ", " + str(x))

    def is_obstacle(self, x, y, sim=True, use_confidence=False):
        if sim:
            return map_sim[y][x] == 1

        if use_confidence:
            return map_virtual_w[y][x] >= 1000

        return map_virtual[y][x] == 1

    def is_free(self, x, y, sim=True):
        return not self.is_obstacle(x, y, sim)

    def valid_range(self, y, x):
        return (0 <= y < config.map_size['height']) and (0 <= x < config.map_size['width'])

    def set_map(self, y, x, stat):
        if not self.valid_range(y, x):
            # logging.debug( "Warning: map to be set is out of bound!", tag="Map", lv='debug' )
            return

        if (stat in self.mapStat):
            self.__map[y][x] = self.mapStat.index(stat)
        # else:
        #     logging.debug( "Error: set map wrong status!", tag="Map", lv='quiet' )

    def mark_explored(self, x, y, is_explored, is_obstacle, is_sim):
        try:
            map_is_explored[y][x] = is_explored

            if (x in [0, 1, 2] and y in [17, 18, 19]) or (x in [12, 13, 14] and y in [0, 1, 2]):
                return

            if is_sim:
                map_virtual[y][x] = is_obstacle
            else:
                map_virtual_w[y][x] += is_obstacle

                map_virtual[y][x] = 1 if map_virtual_w[y][x] > 0 else 0
                # map_virtual[y][x] = is_obstacle

        except IndexError:
            pass

    def set_virtual_wall_around(self, x, y):
        for i in range(3):
            for j in range(3):
                if (self.valid_range(y + j - 1, x + i - 1) and map_virtual[y + j - 1][x + i - 1] == 0):
                    # logging.debug(y + j - 1, x + i - 1)
                    map_virtual[y + j - 1][x + i - 1] = 2

    def set_virtual_wall_border(self):
        for y in range(config.map_size['height']):
            if (map_virtual[y][0] != 1):
                map_virtual[y][0] = 2
            if (map_virtual[y][config.map_size['width'] - 1] != 1):
                map_virtual[y][config.map_size['width'] - 1] = 2

        for x in range(config.map_size['width']):
            if (map_virtual[0][x] != 1):
                map_virtual[0][x] = 2
            if (map_virtual[config.map_size['height'] - 1][x] != 1):
                map_virtual[config.map_size['height'] - 1][x] = 2

    def is_virtual_wall(self, x, y):
        return map_virtual[y][x] == 2

    def is_valid_open(self, x, y):
        return map_virtual[y][x] == 0 and map_is_explored[y][x] == 1

    def is_physical_wall(self, x, y):
        return map_virtual[y][x] == 1

    def get_coverage(self):
        flattened = [i for sub in map_is_explored for i in sub]

        return (sum(flattened) / (config.map_size['height'] * config.map_size['width'])) * 100

    def create_map_descriptor(self):
        explored_str = [str(i) for sub in reversed(map_is_explored) for i in sub]
        explored_str.insert(0, '1')
        explored_str.insert(0, '1')
        explored_str.append('1')
        explored_str.append('1')
        explored_str = "".join(explored_str)

        obstacles_str = []
        reversed_list = list(reversed(map_virtual))

        for y, row in enumerate(reversed(map_is_explored)):
            for x, val in enumerate(row):
                if val:
                    obstacles_str.append(str(reversed_list[y][x]))

        padding_len = 8 - (len(obstacles_str) % 8)
        obstacles_str.extend(['0' for _ in range(padding_len)])

        explored_hex = hex(int(explored_str, 2))[2:]

        i = 0
        obstacles_hex = []

        while i < len(obstacles_str) - 4:
            obstacles_hex.append(hex(int("".join(obstacles_str[i:i + 4]), 2))[2:])
            i += 4

        obstacles_hex = ("".join(obstacles_hex))

        return explored_hex, obstacles_hex

    def decode_map_descriptor(self, obstacles_hex):
        map_bin = []
        size = config.map_size['height'] * config.map_size['width']

        for hex in obstacles_hex:
            map_bin.extend(bin(int(hex, 16))[2:].zfill(4))

        map_bin = [int(x) for x in map_bin]

        map_bin = np.array(list(reversed(map_bin)))[:size].reshape(
            (config.map_size['height'], config.map_size['width'])
        )

        for i in range(len(map_bin)):
            map_bin[i] = list(reversed(map_bin[i]))

        for x in range(config.map_size['width']):
            for y in range(config.map_size['height']):
                map_sim[y][x] = map_bin[y][x]
                map_virtual[y][x] = map_bin[y][x]

        logging.debug(map_sim)

    def clear_map_for_real_exploration(self):
        for x in range(config.map_size['width']):
            for y in range(config.map_size['height']):
                map_sim[y][x] = 0

    def reset(self):
        # map_virtual = [[0 for x in range(config.map_size['width'])] for y in range(config.map_size['height'])]
        # map_is_explored = [[0 for x in range(config.map_size['width'])] for y in range(config.map_size['height'])]

        for x in range(config.map_size['width']):
            for y in range(config.map_size['height']):
                map_virtual[y][x] = 0
                map_is_explored[y][x] = 0

        # assuming robot always start at the start position
        for i in range(3):
            for j in range(3):
                map_is_explored[config.map_size['height'] - 1 - i][j] = 1
                map_is_explored[i][config.map_size['width'] - 1 - j] = 1

    def get_unexplored_grid(self):
        for i in range(config.map_size['height']):
            for j in range(config.map_size['width']):
                if (map_is_explored[config.map_size['height'] - i - 1][j] == 0):
                    return j, config.map_size['height'] - i - 1

    def is_free_space(self, x, y):
        try:
            for i in range(-1, 2):
                for j in range(-1, 2):
                    # logging.debug(map_virtual[y + j][x + i])
                    if not self.is_explored(x + i, y + j) or map_virtual[y + j][x + i] != 0:
                        return False
            return True
        except IndexError:
            pass

    def find_adjacent_free_space(self, x, y):
        center = {
            0: [x + 2, y],
            1: [x - 2, y],
            2: [x, y + 2],
            3: [x, y - 2],
        }

        # check right
        if self.is_free_space(center[0][0], center[0][1]):
            return center[0]

        # check left
        if self.is_free_space(center[1][0], center[1][1]):
            return center[1]

        # check bottom
        if self.is_free_space(center[2][0], center[2][1]):
            return center[2]

        # check top
        if self.is_free_space(center[3][0], center[3][1]):
            return center[3]

        # if cant find, will return None

    def get_unexplored_grids(self):
        unexplored_grids = []
        for j in range(config.map_size['width']):
            for i in range(config.map_size['height']):
                if (map_is_explored[i][config.map_size['width'] - j - 1] == 0):
                    unexplored_grids.append((config.map_size['width'] - j - 1, i))
        return unexplored_grids

    def find_adjacent_free_space_front(self, x, y, ir=False):
        center = {
            Bearing.NORTH: [[x - 1, y + 2], [x, y + 2], [x + 1, y + 2]],
            Bearing.EAST: [[x - 2, y - 1], [x - 2, y], [x - 2, y + 1]],
            Bearing.SOUTH: [[x - 1, y - 2], [x, y - 2], [x + 1, y - 2]],
            Bearing.WEST: [[x + 2, y - 1], [x + 2, y], [x + 2, y + 1]]
        }
        if ir:
            center = {
                Bearing.SOUTH: [[x, y - 2]],
                Bearing.EAST: [[x - 2, y]],
                Bearing.NORTH: [[x, y + 2]],
                Bearing.WEST: [[x + 2, y]]
            }

        for k, v in center.items():
            for e in v:
                # logging.debug("coordinates: ", e, k)
                if e[0] > 0 and e[0] < config.map_size['width'] - 1 and e[1] > 0 and e[1] < config.map_size[
                    'height'] - 1 and self.is_free_space(e[0], e[1]):
                    return e, k

    def find_left_wall_or_obstacle(self, x, y, bearing):
        try:
            left_wall = False

            if bearing == Bearing.NORTH:
                if x == 1:
                    left_wall = True
            elif bearing == Bearing.EAST:
                if y == 1:
                    left_wall = True
            elif bearing == Bearing.SOUTH:
                if x == 13:
                    left_wall = True
            else:
                if y == 18:
                    left_wall = True

            if left_wall:
                return True

            left_obstacle = False

            if bearing == Bearing.NORTH:
                if self.is_explored(x - 2, y - 1) and self.is_obstacle(x - 2, y - 1, False) and \
                        self.is_explored(x - 2, y) and self.is_obstacle(x - 2, y, False):
                    # self.is_explored(x - 2, y + 1) and self.is_obstacle(x - 2, y + 1):
                    left_obstacle = True

            elif bearing == Bearing.EAST:
                # if self.is_explored(x - 1, y - 2) and self.is_obstacle(x - 1, y - 2) and \
                if self.is_explored(x, y - 2) and self.is_obstacle(x, y - 2, False) and \
                        self.is_explored(x + 1, y - 2) and self.is_obstacle(x + 1, y - 2, False):
                    left_obstacle = True

            elif bearing == Bearing.SOUTH:
                # if self.is_explored(x + 2, y - 1) and self.is_obstacle(x + 2, y - 1) and \
                if self.is_explored(x + 2, y) and self.is_obstacle(x + 2, y, False) and \
                        self.is_explored(x + 2, y + 1) and self.is_obstacle(x + 2, y + 1, False):
                    left_obstacle = True

            else:
                if self.is_explored(x - 1, y + 2) and self.is_obstacle(x - 1, y + 2, False) and \
                        self.is_explored(x, y + 2) and self.is_obstacle(x, y + 2, False):
                    # self.is_explored(x + 1, y + 2) and self.is_obstacle(x + 1, y + 2):
                    left_obstacle = True

            return left_obstacle
        except IndexError:
            logging.debug('Index error during calibration check')
            pass


map_partial_explored = \
    [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], ]