import math
import sys
import time

from queue import PriorityQueue
from constants import *
from fastest_path_algo import FastestPath
from map import *
from path_find_algo import *
from setup_logger import logger


class Robot:
    def __init__(self, simulator):
        self.simulator = simulator
        self.map: Map = Map()
        
        # self.bearing: Bearing = Bearing.NORTH
        # self.prev_loc = (2, 9, Bearing.NORTH)  # (x, y, Bearing)
        
        self.y: int = 9  # robot initial position
        self.x: int = 2
        self.bearing: Bearing = Bearing.NORTH
        self.prev_loc = (2, 9, Bearing.NORTH)  # (x, y, Bearing)
        
        self.consecutive_forward: int = 1
        self.update_map: bool = True
        self.robot_rpi_temp_movement: List[str] = []

    def validate(self, x, y):
        if (
            0 < self.x + x < config.map_size["width"] - 1
            and 0 < self.y + y < config.map_size["height"] - 1
        ):
            return True

    def set_location(self, x, y):
        self.x = x
        self.y = y

    # recalculate center of robot
    def move(self):
        if self.bearing == Bearing.NORTH and self.check_front():
            self.y -= 1
        elif self.bearing == Bearing.EAST and self.check_front():
            self.x += 1
        elif self.bearing == Bearing.SOUTH and self.check_front():
            self.y += 1
        elif self.bearing == Bearing.WEST and self.check_front():
            self.x -= 1

    def reverse(self):
        if self.bearing == Bearing.NORTH:
            self.y += 1
        elif self.bearing == Bearing.EAST:
            self.x -= 1
        elif self.bearing == Bearing.SOUTH:
            self.y -= 1
        elif self.bearing == Bearing.WEST:
            self.x += 1

    def left(self):
        # rotate anticlockwise by 90 deg
        self.bearing = Bearing.prev_bearing(self.bearing)

    def right(self):
        # rotate clockwise by 90 deg
        self.bearing = Bearing.next_bearing(self.bearing)

    def get_right_bearing(self):
        return Bearing.next_bearing(self.bearing)

    def get_left_bearing(self):
        return Bearing.prev_bearing(self.bearing)

    def get_back_bearing(self):
        return Bearing.next_bearing(Bearing.next_bearing(self.bearing))

    def reset(self):
        # self.y = config.map_size["height"] - 2
        # self.x = 1
        # self.bearing = Bearing.NORTH
        # self.prev_loc = (1, 18, Bearing.NORTH)
        
        self.y: int = 9  # robot initial position
        self.x: int = 2
        self.bearing: Bearing = Bearing.NORTH
        self.prev_loc = (2, 9, Bearing.NORTH)

    def check_front(self):
        if (
            self.bearing == Bearing.NORTH
            and self.validate(0, -1)
            and self.north_is_free()
        ):
            return True
        elif (
            self.bearing == Bearing.EAST 
            and self.validate(1, 0) 
            and self.east_is_free()
        ):
            return True
        elif (
            self.bearing == Bearing.SOUTH
            and self.validate(0, 1)
            and self.south_is_free()
        ):
            return True
        elif (
            self.bearing == Bearing.WEST
            and self.validate(-1, 0)
            and self.west_is_free()
        ):
            return True
        else:
            return False

    # check obstacles
    def north_is_free(self):
        for i in range(3):
            if map_sim[self.y - 2][self.x - i + 1] in [10, 11, 12, 13]:
                return False
        return True

    def south_is_free(self):
        for i in range(3):
            if map_sim[self.y + 2][self.x - i + 1] in [10, 11, 12, 13]:
                return False
        return True

    def east_is_free(self):
        for i in range(3):
            if map_sim[self.y - i + 1][self.x + 2] in [10, 11, 12, 13]:
                return False
        return True

    def west_is_free(self):
        for i in range(3):
            if map_sim[self.y - i + 1][self.x - 2] in [10, 11, 12, 13]:
                return False
        return True

    def get_target_movement(self, from_dir: Bearing, to_dir) -> None:
        if from_dir == to_dir:
            return

        movements = {
            Bearing.NORTH: {
                Bearing.EAST: [Movement.RIGHT],
                Bearing.SOUTH: [Movement.RIGHT] * 2,
                Bearing.WEST: [Movement.LEFT],
            },
            Bearing.EAST: {
                Bearing.SOUTH: [Movement.RIGHT],
                Bearing.WEST: [Movement.RIGHT] * 2,
                Bearing.NORTH: [Movement.LEFT],
            },
            Bearing.SOUTH: {
                Bearing.WEST: [Movement.RIGHT],
                Bearing.NORTH: [Movement.RIGHT] * 2,
                Bearing.EAST: [Movement.LEFT],
            },
            Bearing.WEST: {
                Bearing.NORTH: [Movement.RIGHT],
                Bearing.EAST: [Movement.RIGHT] * 2,
                Bearing.SOUTH: [Movement.LEFT],
            },
        }

        self.simulator.robot_movement.extend(movements[from_dir][to_dir])
        self.robot_rpi_temp_movement.extend(movements[from_dir][to_dir])
        self.bearing = Bearing.int_to_bearing(to_dir)



    def fastestPath(self, maze):
        self.simulator.temp_pairs = []
        start = [2, 9, 11]
        
        target_states = []
        g = self.simulator.goal_pairs
        g.insert(0, start)
        encoded_pairs = {}
        count = 0
        k = []
        for obstacle in self.simulator.obstacles:
            if obstacle.direction == 10:
                k.append(
                    [
                        obstacle.x,
                        obstacle.y - Distance.IMAGE_CAPTURE.value,
                        12,
                    ]
                )
            elif obstacle.direction == 11:
                k.append(
                    [
                        obstacle.x + Distance.IMAGE_CAPTURE.value,
                        obstacle.y,
                        13,
                    ]
                )
            elif obstacle.direction == 12:
                k.append(
                    [
                        obstacle.x,
                        obstacle.y + Distance.IMAGE_CAPTURE.value,
                        10,
                    ]
                )
            else:
                k.append(
                    [
                        obstacle.x - Distance.IMAGE_CAPTURE.value,
                        obstacle.y,
                        11,
                    ]
                )

        for i in g:
            encoded_pairs[count] = i
            count += 1

        self.encoded_pairs = encoded_pairs
        logger.debug(f"encoded_pairs: {encoded_pairs}")
        dist = []
        for i in g:
            temp = []
            for j in g:
                if i == j:
                    temp.append(sys.maxsize)
                else:
                    sqr = pow(i[0] - j[0], 2) + pow(i[1] - j[1], 2)
                    root = math.sqrt(sqr)
                    temp.append(root)
            dist.append(temp)
        n = len(g)
        fastest_path = FastestPath()
        path = fastest_path.plan_path(dist, n)
        logger.debug(path)
        for i in path:
            if i != 0:
                target_states.append(encoded_pairs[i])
                t = encoded_pairs[i]
                for j in range(len(k)):
                    if k[j][0] == t[0] and k[j][1] == t[1]:
                        break
        for x in target_states:
            if x[2] == 10:
                tempGoal = [x[0], x[1] - Distance.IMAGE_CAPTURE.value]
            elif x[2] == 11:
                tempGoal = [x[0] + Distance.IMAGE_CAPTURE.value, x[1]]
            elif x[2] == 12:
                tempGoal = [x[0], x[1] + Distance.IMAGE_CAPTURE.value]
            else:
                tempGoal = [x[0] - Distance.IMAGE_CAPTURE.value, x[1]]
            self.simulator.temp_pairs.append(tempGoal)

        self.hamiltonian_path_search(maze, target_states)
        
        """RETURN TO STARTING POINT/ BACK TO CARPARK"""
        # Convert your final obstacle position and start position to grid coordinates
        # final_obstacle_position = (self.x, self.y)  # Assuming this is set to the last obstacle's position
        # start_position = (1, 18)  # Back to start; adjust if your grid's origin differs
        
        # start_position = (1, 18)  # Update with actual variable names
        # current_position = (self.x, self.y)

        # # Call A* to get the shortest path back to start
        # # return_path = self.a_star_search(final_obstacle_position, start_position, self.map)
        # return_path = self.a_star_search(current_position, start_position, self.map)
        
        # # Append the return path movements to the robot's command list
        # # You will need to convert the grid path back to your robot's movement commands
        # # This part highly depends on how your robot interprets movements
        # for step in return_path:
        #     # Example conversion, implement based on your movement system
        #     self.move_to(step)  # You need to implement move_to or a similar method
        

    ########################################################################################

    def hamiltonian_path_search(self, maze, target_states):
        # start = [18, 1, 10]
        start = [9, 2, 11]
        
        end = [
            target_states[0][1],
            target_states[0][0],
            target_states[0][2],
        ]  # ending position
        
        cost = 10  # cost per movement
        for i in range(len(target_states)):
            self.simulator.robot_temp_movement = []
            self.robot_rpi_temp_movement = []
            path = search(maze, cost, start, end, self.simulator.obstacles)

            # Path movement
            num_moves = len([num for row in path for num in row if num not in [-1, 0]])
            self.simulator.robot_temp_movement = [None] * num_moves
            for row in range(len(path)):
                for item in range(len(path[row])):
                    if path[row][item] not in [-1, 0]:
                        self.simulator.robot_temp_movement[path[row][item] - 1] = [
                            row,
                            item,
                        ]

            tempStart = start
            for j in range(len(self.simulator.robot_temp_movement)):
                move = [
                    [tempStart[0] - 1, tempStart[1]],  # N
                    [tempStart[0], tempStart[1] + 1],  # E
                    [tempStart[0] + 1, tempStart[1]],  # S
                    [tempStart[0], tempStart[1] - 1],  # W
                ]
                direction = [Bearing.NORTH, Bearing.EAST, Bearing.SOUTH, Bearing.WEST]

                # Find the ONE AND ONLY coordinate in `move` which matches self.simulator.robot_temp_movement[j]
                for k in range(len(move)):
                    if move[k] in self.simulator.robot_temp_movement:
                        if self.bearing == direction[k]:
                            self.simulator.robot_movement.append(Movement.FORWARD)
                            self.robot_rpi_temp_movement.append(Movement.FORWARD)
                            tempStart = move[k]
                            self.simulator.robot_temp_movement.remove(tempStart)
                            break
                        elif self.bearing == direction[(k + 2) % 4]:
                            self.simulator.robot_movement.append(Movement.REVERSE)
                            self.robot_rpi_temp_movement.append(Movement.REVERSE)
                            tempStart = move[k]
                            self.simulator.robot_temp_movement.remove(tempStart)
                            break
                        elif self.bearing == direction[(k + 1) % 4]:
                            self.simulator.robot_movement.append(Movement.LEFT)
                            self.simulator.robot_movement.append(Movement.FORWARD)
                            self.robot_rpi_temp_movement.append(Movement.LEFT)
                            self.robot_rpi_temp_movement.append(Movement.FORWARD)
                            self.bearing = Bearing.prev_bearing(self.bearing)
                            tempStart = move[k]
                            self.simulator.robot_temp_movement.remove(tempStart)
                            break
                        elif self.bearing == direction[(k + 3) % 4]:
                            self.simulator.robot_movement.append(Movement.RIGHT)
                            self.simulator.robot_movement.append(Movement.FORWARD)
                            self.robot_rpi_temp_movement.append(Movement.RIGHT)
                            self.robot_rpi_temp_movement.append(Movement.FORWARD)
                            self.bearing = Bearing.next_bearing(self.bearing)
                            tempStart = move[k]
                            self.simulator.robot_temp_movement.remove(tempStart)
                            break

            self.get_target_movement(
                self.bearing, Bearing.conversion_robot(target_states[i][2])
            )
            self.simulator.robot_movement.append(Movement.STOP)
            self.robot_rpi_temp_movement.append(Movement.STOP)
            self.simulator.movement_to_rpi.append(self.robot_rpi_temp_movement)
            start = end
            if i + 1 < len(target_states):
                end = [
                    target_states[i + 1][1],
                    target_states[i + 1][0],
                    target_states[i + 1][2],
                ]

        """change"""
        self.bearing = Bearing.NORTH  # Reset bearing to North
        self.displayMovement()  # TODO - this is removing my first element in self.simulator.robot_movement()

    def displayMovement(self):
        if not self.simulator.robot_movement:
            return
        movement = self.simulator.robot_movement.pop(0)
        if movement == Movement.FORWARD:
            self.move()
        elif movement == Movement.LEFT:
            self.left()
        elif movement == Movement.RIGHT:
            self.right()
        elif movement == Movement.REVERSE:
            self.reverse()
        elif movement == Movement.STOP:
            goal = self.simulator.temp_pairs.pop(0)
            map_sim[goal[1]][goal[0]] = 1
            time.sleep(0.5)
        self.simulator.update_map(full=True)
        # Refresh every 0.5 sec
        self.simulator.job = self.simulator.root.after(50, self.displayMovement)