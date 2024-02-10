import logging

import config
from constants import *
from map import *


class Robot:
    def __init__(self, handler):
        # center of robot
        self.y = config.map_size['height'] - 2
        self.x = 1
        self.bearing = Bearing.NORTH
        self.map = handler.map
        self.handler = handler
        self.update_map = True
        self.just_turn = False
        self.consecutive_forward = 1
        # self.ir_current_island = True
        self.map_img_rec = [[0 for _ in range(config.map_size['width'])] for _ in range(config.map_size['height'])]
        self.prev_loc = [((1,18),Bearing.NORTH)] # tuple of location coordinates and bearing

    # check that center of robot is not at the border and lies within the map
    def validate(self, x, y):
        if 0 < self.x + x < config.map_size['width'] - 1 and 0 < self.y + y < config.map_size['height'] - 1:
            return True

    # recalculate center of robot
    def move(self, sense, ir, steps=1):
        logging.debug('f' + str(steps))
        self.consecutive_forward += 1
        for _ in range(steps):
            if self.bearing == Bearing.NORTH:
                if self.validate(0, -1) and self.north_is_free():
                    self.y -= 1
            elif self.bearing == Bearing.EAST:
                if self.validate(1, 0) and self.east_is_free():
                    self.x += 1
            elif self.bearing == Bearing.SOUTH:
                if self.validate(0, 1) and self.south_is_free():
                    self.y += 1
            else:
                if self.validate(-1, 0) and self.west_is_free():
                    self.x -= 1
            if sense:
                self.sense()
                self.handler.simulator.update_map(radius=3)
            # self.add_prev_location()
        if ir:
            self.take_image()
        self.handler.simulator.update_map(radius=3)


    def left(self, sense, ir):
        if ir:
            self.take_image(before_turn=True)
        # rotate anticlockwise by 90 deg
        logging.debug('l90')
        self.bearing = Bearing.prev_bearing(self.bearing)
        self.just_turn = True
        if sense:
            self.sense()
        if ir:
            self.take_image()
        self.handler.simulator.update_map(radius=3)
        # self.add_prev_location()

    def right(self, sense, ir):
        if ir:
            self.take_image(before_turn=True)
        logging.debug('r90')
        # rotate clockwise by 90 deg
        self.bearing = Bearing.next_bearing(self.bearing)
        self.just_turn = True
        if sense:
            self.sense()
        if ir:
            self.take_image()
        self.handler.simulator.update_map(radius=3)
        # self.add_prev_location()

    def left_diag(self):
        logging.debug('l33')
        self.bearing = Bearing.prev_bearing_diag(self.bearing)
        # self.add_prev_location()

    def right_diag(self):
        logging.debug('r33')
        self.bearing = Bearing.next_bearing_diag(self.bearing)
        # self.add_prev_location()

    def move_diag(self, steps=1):
        logging.debug('h' + str(steps))
        if self.bearing == Bearing.NORTH_EAST:
            self.x += steps
            self.y -= steps
        elif self.bearing == Bearing.SOUTH_EAST:
            self.x += steps
            self.y += steps
        elif self.bearing == Bearing.SOUTH_WEST:
            self.x -= steps
            self.y += steps
        else:
            self.x -= steps
            self.y -= steps
        # self.add_prev_location()

    def calibrate(self):
        logging.debug("CALIBRATE")

    def signal_exploration_ended(self):
        return

    # check obstacles
    def north_is_free(self):
        for i in range(3):
            if map_virtual[self.y - 2][self.x - i + 1] == 1:
                return False
        return True

    def south_is_free(self):
        for i in range(3):
            if map_virtual[self.y + 2][self.x - i + 1] == 1:
                return False
        return True

    def east_is_free(self):
        for i in range(3):
            if map_virtual[self.y - i + 1][self.x + 2] == 1:
                return False
        return True

    def west_is_free(self):
        for i in range(3):
            if map_virtual[self.y - i + 1][self.x - 2] == 1:
                return False
        return True

    def get_location(self):
        return self.x, self.y

    def set_location(self, x, y):
        self.x = x
        self.y = y

    def get_right_bearing(self):
        return Bearing.next_bearing(self.bearing)

    def get_left_bearing(self):
        return Bearing.prev_bearing(self.bearing)

    def get_back_bearing(self):
        return Bearing.next_bearing(Bearing.next_bearing(self.bearing))

    def receive(self):
        raise NotImplementedError

    def sense_front(self, location, bearing, sensor_data):

        sensor_offsets = [
            [[-1, 0, 1], [-1, -1, -1]],
            [[1, 1, 1], [-1, 0, 1]],
            [[1, 0, -1], [1, 1, 1]],
            [[-1, -1, -1], [1, 0, -1]]
        ]
        offset = sensor_offsets[int(bearing / 2)]

        self.handler.update_map(location[0] + offset[0][0], location[1] + offset[1][0], sensor_data[0],
                                bearing, config.sensor_range['front_left'])
        self.handler.update_map(location[0] + offset[0][1], location[1] + offset[1][1], sensor_data[1],
                                bearing, config.sensor_range['front_middle'])
        self.handler.update_map(location[0] + offset[0][2], location[1] + offset[1][2], sensor_data[2],
                                bearing, config.sensor_range['front_right'])

    # call update map for left sensor
    def sense_left(self, location, bearing, sensor_data):

        sensor_offsets = [
            [[-1, -1], [-1, 0]],
            [[1, -1], [0, -1]],
            [[1, 1], [1, 0]],
            [[-1, 1], [0, 1]]
        ]
        offset = sensor_offsets[int(bearing / 2)]
        self.handler.update_map(location[0] + offset[0][0], location[1] + offset[0][1], sensor_data[0],
                                Bearing.prev_bearing(bearing), config.sensor_range['left_front'])
        self.handler.update_map(location[0] + offset[1][0], location[1] + offset[1][1], sensor_data[1],
                                Bearing.prev_bearing(bearing), config.sensor_range['left_middle'])

    # call update map for right sensor
    def sense_right(self, location, bearing, sensor_data):

        sensor_offsets = [
            [1, 0],
            [0, 1],
            [-1, 0],
            [0, -1]
        ]
        offset = sensor_offsets[int(bearing / 2)]

        self.handler.update_map(location[0] + offset[0], location[1] + offset[1], sensor_data,
                                Bearing.next_bearing(bearing), config.sensor_range['right'])

    # sense simulated sensor
    def sense(self, backtrack=0):
        sensor_data = self.receive()

        location = self.get_location()
        bearing = self.bearing

        if self.update_map:
            self.sense_front(location, bearing, sensor_data[:3])
            self.sense_left(location, bearing, sensor_data[3:5])
            self.sense_right(location, bearing, sensor_data[5])

        for i in range(1, backtrack + 1):
            if bearing == Bearing.NORTH:
                x, y = location
                y += i
            elif bearing == Bearing.SOUTH:
                x, y = location
                y -= i
            elif bearing == Bearing.EAST:
                x, y = location
                x -= i
            else:
                x, y = location
                x += i

            # self.set_location(x, y)
            # sensor_data = self.receive()
            # self.sense_front((x, y), bearing, sensor_data[:3])
            # self.sense_left((x, y), bearing, sensor_data[3])
            # self.sense_right((x, y), bearing, sensor_data[4])

        x, y = location
        self.set_location(x, y)

    def reset(self):
        self.y = config.map_size['height'] - 2
        self.x = 1
        self.bearing = Bearing.NORTH
        self.just_turn = False
        self.prev_loc = [((1,18),Bearing.NORTH)]
        for i in range(config.map_size['height']):
            for j in range(config.map_size['width']):
                self.map_img_rec[i][j] = 0

    def check_front(self):
        try:
            robot_front = [
                [[self.x - 1, self.y - 2], [self.x, self.y - 2], [self.x + 1, self.y - 2]],
                [[self.x + 2, self.y - 1], [self.x + 2, self.y], [self.x + 2, self.y + 1]],
                [[self.x + 1, self.y + 2], [self.x, self.y + 2], [self.x - 1, self.y + 2]],
                [[self.x - 2, self.y + 1], [self.x - 2, self.y], [self.x - 2, self.y - 1]]
            ]
            offset = [
                [0, -1],
                [1, 0],
                [0, 1],
                [-1, 0]
            ]
            if self.bearing == Bearing.NORTH:
                front = robot_front[0]
                ofs = offset[0]
            elif self.bearing == Bearing.EAST:
                front = robot_front[1]
                ofs = offset[1]
            elif self.bearing == Bearing.SOUTH:
                front = robot_front[2]
                ofs = offset[2]
            else:
                front = robot_front[3]
                ofs = offset[3]
            front_free = []
            for grid in front:
                free = 0
                for i in range(3):
                    if not self.map.valid_range(grid[1] + i * ofs[1], grid[0] + i * ofs[0]) or \
                            not self.map.is_explored(grid[0] + i * ofs[0], grid[1] + i * ofs[1]) or \
                            self.map.is_obstacle(grid[0] + i * ofs[0], grid[1] + i * ofs[1], sim=False):
                        break
                    free += 1
                front_free.append(free)
            return front_free
        except IndexError:
            pass

    def take_image(self, before_turn=False):
        robot_x, robot_y = self.get_location()
        robot_bearing = self.bearing
        for i in range(3):
            for j in range(3):
                if self.map.valid_range(robot_y + i - 1, robot_x + j - 1):
                    self.map_img_rec[robot_y + i - 1][robot_x + j - 1] = 1
        if robot_x < 2 and robot_bearing == Bearing.NORTH:
            return
        if robot_y < 2 and robot_bearing == Bearing.EAST:
            return
        if robot_x > config.map_size['width'] - 3 and robot_bearing == Bearing.SOUTH:
            return
        if robot_y > config.map_size['height'] - 3 and robot_bearing == Bearing.WEST:
            return

        img_target = [
            [[robot_x - 2, robot_y - 1], [robot_x - 2, robot_y], [robot_x - 2, robot_y + 1]],
            [[robot_x + 1, robot_y - 2], [robot_x, robot_y - 2], [robot_x - 1, robot_y - 2]],
            [[robot_x + 2, robot_y + 1], [robot_x + 2, robot_y], [robot_x + 2, robot_y - 1]],
            [[robot_x - 1, robot_y + 2], [robot_x, robot_y + 2], [robot_x + 1, robot_y + 2]]
        ]
        img_pos = img_target[int(robot_bearing // 2)]

        if self.map.valid_range(img_pos[1][1], img_pos[1][0]) and (
                self.map.is_obstacle(img_pos[0][0], img_pos[0][1], False) or \
                self.map.is_obstacle(img_pos[1][0], img_pos[1][1], False) or \
                self.map.is_obstacle(img_pos[2][0], img_pos[2][1], False)):
            self.map_img_rec[img_pos[0][1]][img_pos[0][0]] = 1
            self.map_img_rec[img_pos[1][1]][img_pos[1][0]] = 1
            self.map_img_rec[img_pos[2][1]][img_pos[2][0]] = 1

            for i in range(3):
                if not self.map.is_obstacle(img_pos[i][0], img_pos[i][1], sim=False):
                    img_pos[i] = [-1, -1]
                else:
                    img_pos[i][1] = 19 - img_pos[i][1]

            # if img_pos[0] == [-1, -1] and img_pos[1] == [-1, -1] and img_pos[2] == [-1, -1]:
                # self.ir_current_island = True
                # self.consecutive_forward = 1
            # if not self.ir_current_island:
            #     return

            if self.just_turn or before_turn or self.handler.core.explorer.optimized or\
                    (self.consecutive_forward == 4 and not self.handler.core.explorer.optimized):
                if self.just_turn:
                    self.just_turn = False
                self.consecutive_forward = 1
                logging.debug("[ROBOT] Image taken at {}, {}".format(img_pos[0][0], img_pos[0][1]))  # take photo command
                logging.debug("[ROBOT] Image taken at {}, {}".format(img_pos[1][0], img_pos[1][1]))  # take photo command
                logging.debug("[ROBOT] Image taken at {}, {}".format(img_pos[2][0], img_pos[2][1]))  # take photo command
                logging.debug("\n")
                return img_pos[2], img_pos[1], img_pos[0]

    def execute_fastest_path(self, movements):
        num_move = 1
        movement = movements.pop(0)
        if movement == MOVEMENT.FORWARD or movement == MOVEMENT.FORWARD_DIAG:
            while len(movements) > 0:
                if movements[0] == movement:
                    movements.pop(0)
                    num_move += 1
                else:
                    break
        if (movement == MOVEMENT.LEFT):
            self.handler.left(sense=False, ir=False)
        elif (movement == MOVEMENT.RIGHT):
            self.handler.right(sense=False, ir=False)
        elif (movement == MOVEMENT.LEFT_DIAG):
            self.handler.left_diag()
        elif (movement == MOVEMENT.RIGHT_DIAG):
            self.handler.right_diag()
        elif (movement == MOVEMENT.FORWARD_DIAG):
            self.handler.move_diag(steps=num_move)
        else:
            self.handler.move(steps=num_move, sense=False, ir=False)

        if len(movements) > 0:
            self.handler.simulator.job = self.handler.simulator.root.after(1000, self.execute_fastest_path, movements)

    def stop_ir_current_island(self):
        # self.ir_current_island = False
        self.consecutive_forward = 1

    def add_prev_location(self):
        self.prev_loc.append(((self.x, self.y), self.bearing))

    def revert_loop(self):
        for _ in range(10):
            if len(self.prev_loc) > 1:
                self.prev_loc.pop(len(self.prev_loc) - 1)
            else:
                return self.prev_loc[0]
        return self.prev_loc[len(self.prev_loc) - 1]

    def pop_prev_loc(self):
        self.prev_loc.pop(len(self.prev_loc) - 1)