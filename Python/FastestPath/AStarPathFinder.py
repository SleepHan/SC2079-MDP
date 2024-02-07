"""
fastest car task using visual recognition 

Map : 200 by 200

Robot has a footprint of 20cm x 21cm

The best position is to have the camera 20cm away from the obstacle to recognize an image.

obstacle will be placed at the centre of the carpark
- obstacle 1 : 10 x 10
- obstacle 2 : 60 x 10

parking barrier will be 60cm wide

wall/barrier is approximately 50cm away from obstacle 2

right / left arrow only

Time out for this task is 3min.
"""

# imports
from enum import Enum
import math
import numpy as np


class Directions(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    FORWARD = 4
    LEFT = 5
    RIGHT = 6
    BACKWARD = 7


class Position(Enum):
    X = 0
    Y = 1


class ArenaConfig:
    ARENA_WIDTH = 15
    ARENA_HEIGHT = 20


class ArenaMap:
    def __init__(self):
        # Implement ArenaMap class as needed
        pass


class Robot:
    def __init__(self, map):
        self.map = map


class Node:
    def __init__(self, position):
        self.position = position
        self.g = 0
        self.h = 0
        self.cost = self.g + self.h
        self.parent = None

    def update_cost(self):
        self.cost = self.h + self.g

    def get_g(self):
        return self.g

    def set_g(self, g):
        self.g = g

    def get_h(self):
        return self.h

    def set_h(self, h):
        self.h = h

    def get_cost(self):
        return self.cost

    def set_cost(self, cost):
        self.cost = cost

    def get_position(self):
        return self.position

    def set_position(self, position):
        self.position = position

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent


class AStarPathFinder:
    def __init__(self):
        self.first = True
        self.direction = -1
        self.first_penalty = True

    def start(self, robot, starting_position, goal_position, on_grid):
        """the open_list contains nodes that are candidates for further exploration, 
        and the closed_list contains nodes that have already been explored.
        The loop iterates by selecting the node with the lowest cost from the open_list, 
        exploring its neighbors, and updating the lists accordingly.
        The method returns the found path."""
        start_node = Node(starting_position)
        current_node = None
        open_set = [start_node]
        closed_set = []

        while True:
            current_node = self.check_lowest_cost(open_set)

            if current_node is None:
                print("Error: open is empty")
                break
            else:
                open_set = self.remove_node(open_set, current_node)
                closed_set = self.add_node(closed_set, current_node)

                if (not on_grid and self.can_reach(current_node.get_position(), goal_position, self.first)) or (
                        on_grid and np.array_equal(current_node.get_position(), goal_position)):
                    print("Path found!")
                    break

                open_set = self.add_neighbours(robot, open_set, current_node, goal_position)

                if len(open_set) == 0:
                    self.set_first(False)
                    print("Error: No possible path")
                    return None

        path = self.get_path(current_node)
        print(np.array(path))
        self.update_direction(path)
        print("Path Found")
        return path

    def update_direction(self, path):
        """updates the direction attribute based on the movements in the path"""
        if path is not None:
            for value in path:
                if value == Directions.LEFT:
                    self.direction = (self.direction + 3) % 4
                elif value == Directions.RIGHT:
                    self.direction = (self.direction + 1) % 4
                elif value == Directions.BACKWARD:
                    self.direction = (self.direction + 2) % 4

    def can_reach(self, current_node_position, end_position, first):
        """Checks if the current_node_position can reach the end_position based on certain conditions"""
        x, y = end_position[0], end_position[1]
        """If first is True, it checks for positions in a specific pattern relative to the end position.
        The conditions are determined by the 'first' parameter."""
        if first:
            positions = np.array([
                [x - 1, y - 2], [x, y - 2], [x + 1, y - 2],
                [x + 2, y - 1], [x + 2, y], [x + 2, y + 1],
                [x + 1, y + 2], [x, y + 2], [x - 1, y + 2],
                [x - 2, y + 1], [x - 2, y], [x - 2, y - 1]
            ])
        
        else: 
            """If first is False, it checks for positions in a different pattern."""
            positions = np.array([
                [x - 1, y - 3], [x, y - 3], [x + 1, y - 3],
                [x + 3, y - 1], [x + 3, y], [x + 3, y + 1],
                [x + 1, y + 3], [x, y + 3], [x - 1, y + 3],
                [x - 3, y + 1], [x - 3, y], [x - 3, y - 1]
            ])
        """The method returns True if current_node_position matches any of the calculated positions, otherwise, it returns False"""
        return any(np.array_equal(current_node_position, coordinates) for coordinates in positions)

    def check_lowest_cost(self, node_list):
        """This method takes a list of nodes (node_list) and finds the node with the lowest cost.
        It iterates through the nodes and updates lowest_cost and lowest_node accordingly.
        The method returns the node with the lowest cost. If the list is empty, it returns None"""
        cost = 0
        if len(node_list) > 0:
            lowest_cost = node_list[0].get_cost()
            lowest_node = node_list[0]

            for node in node_list:
                cost = node.get_cost()
                if cost <= lowest_cost:
                    lowest_cost = cost
                    lowest_node = node
            return lowest_node
        else:
            return None

    def remove_node(self, node_list, node):
        """This method removes a specified node from the node_list.
        If the list has fewer than 2 nodes, it returns an empty list.
        Otherwise, it creates a new list (new_list) by excluding the node at the specified index"""
        index = -1
        if len(node_list) < 2:
            return []
        new_list = node_list[:index] + node_list[index + 1:] if index > -1 else node_list
        return new_list

    def add_node(self, node_list, node):
        return node_list + [node]

    def add_neighbours(self, robot, open_list, current_node, goal_position):
        """This method adds valid neighboring nodes to the open_list for further exploration.
        It calculates positions for potential neighbors based on the current node's position.
        For each valid neighbor, it creates a Node object, sets its parent and cost, and adds it to the neighbours array.
        Finally, it adds the valid neighbors to the open_list using the add_node method"""
        neighbours = [None] * 4
        count = 0
        x, y = current_node.get_position()[Position.X], current_node.get_position()[Position.Y]
        neighbours_positions = [
            [x, y + 1], [x - 1, y], [x + 1, y], [x, y - 1]
        ]

        for i in range(4):
            if self.is_valid(robot, neighbours_positions[i]):
                neighbour = Node(neighbours_positions[i])
                neighbour.set_parent(current_node)
                neighbour.set_cost(self.find_cost(neighbour, goal_position))
                neighbours[count] = neighbour
                count += 1

        for j in range(count):
            node = neighbours[j]
            open_list = self.add_node(open_list, node)

        return open_list

    def is_valid(self, robot, position):
        """This method checks whether a given position is valid based on the robot's current map.
        It considers the robot's position along with the surrounding coordinates.
        If the position is within the map boundaries and does not collide with an obstacle, it returns True; otherwise, it returns False"""
        if position is None:
            return False

        arena_map = robot.map
        x, y = position[Position.X], position[Position.Y]
        robot_pos = [
            [x - 1, y + 1], [x, y + 1], [x + 1, y + 1],
            [x - 1, y], [x, y], [x + 1, y],
            [x - 1, y - 1], [x, y - 1], [x + 1, y - 1]
        ]

        if 0 < x < ArenaConfig.ARENA_WIDTH - 1 and 0 < y < ArenaConfig.ARENA_HEIGHT - 1:
            for coordinates in robot_pos:
                if arena_map.get_grid(coordinates[0], coordinates[1]) == "Obstacle":
                    return False
            return True
        else:
            return False

    def find_cost(self, node, goal_position):
        """This method calculates and sets the total cost of a given node.
        It utilizes the find_h_cost and find_g_cost methods to calculate the heuristic and actual cost, respectively.
        The total cost is then updated and returned"""
        node.set_h(self.find_h_cost(node, goal_position))
        node.set_g(self.find_g_cost(node))
        node.update_cost()
        return node.get_cost()

    def find_h_cost(self, current_node, goal_position):
        """This method calculates the heuristic cost (h) of a given current_node based on its position and the goal position.
        It considers the absolute differences in X and Y coordinates and returns the sum."""
        x = abs(current_node.get_position()[Position.X] - goal_position[Position.X])
        y = abs(current_node.get_position()[Position.Y] - goal_position[Position.Y])

        if current_node.get_parent().get_position()[Position.X] == current_node.get_position()[Position.X] and x == 0:
            return y
        elif current_node.get_parent().get_position()[Position.Y] == current_node.get_position()[Position.Y] and y == 0:
            return x
        else:
            return x + y

    def find_g_cost(self, current_node):
        """This method calculates the actual cost (g) of a given current_node based on its parent and the direction of movement.
        It considers different scenarios, including the first penalty condition.
        The method returns the calculated actual cost."""
        previous_node = current_node.get_parent()

        if previous_node is None:
            return 0
        elif not self.first_penalty and previous_node.get_parent() is None:
            return previous_node.get_g() + 1
        else:
            direction = self.go_where(current_node)
            if direction == Directions.FORWARD:
                return previous_node.get_g() + 1
            elif direction == Directions.LEFT or direction == Directions.RIGHT:
                return previous_node.get_g() + 3
            else:
                return previous_node.get_g() + 5

    def go_where(self, current_node):
        second = current_node.get_parent()
        if second is None:
            return -1
        first = second.get_parent()
        first_node_position, second_node_position = None, None

        if first is None:
            second_node_position = second.get_position()
            """This condition checks whether the X coordinate of second_node_position is equal to the X coordinate of current_node. 
            If they are the same, it means the nodes are aligned vertically"""
            if second_node_position[Position.X] == current_node.get_position()[Position.X]: # Check if the X coordinates are the same
                """this condition checks whether the Y coordinate of second_node_position is greater than the Y coordinate of current_node. 
                If true, it means second_node_position is positioned above current_node"""
                if second_node_position[Position.Y] > current_node.get_position()[Position.Y]: # Check if second_node_position is above current_node
                    return {
                        Directions.NORTH: Directions.FORWARD,
                        Directions.EAST: Directions.LEFT,
                        Directions.SOUTH: Directions.BACKWARD,
                        Directions.WEST: Directions.RIGHT
                    }[self.direction]
                
                elif second_node_position[Position.Y] < current_node.get_position()[Position.Y]:
                    """This condition checks whether the Y coordinate of second_node_position is less than the Y coordinate of current_node"""
                    return {
                        Directions.NORTH: Directions.BACKWARD,
                        Directions.EAST: Directions.RIGHT,
                        Directions.SOUTH: Directions.FORWARD,
                        Directions.WEST: Directions.LEFT
                    }[self.direction]
                
                """condition checks whether the Y coordinate of second_node_position is equal to the Y coordinate of current_node. If true, it means second_node_position is at the same Y coordinate as current_node"""
            elif second_node_position[Position.Y] == current_node.get_position()[Position.Y]:
                """If second_node_position[Position.X] > current_node.get_position()[Position.X]
                means second_node_position is to the right of current_node."""
                if second_node_position[Position.X] > current_node.get_position()[Position.X]:
                    return {
                        Directions.NORTH: Directions.LEFT,
                        Directions.EAST: Directions.BACKWARD,
                        Directions.SOUTH: Directions.RIGHT,
                        Directions.WEST: Directions.FORWARD
                    }[self.direction]
                else:
                    """If second_node_position[Position.X] <= current_node.get_position()[Position.X]
                    means second_node_position is to the left of or at the same X coordinate as current_node."""
                    return {
                        Directions.NORTH: Directions.RIGHT,
                        Directions.EAST: Directions.FORWARD,
                        Directions.SOUTH: Directions.LEFT,
                        Directions.WEST: Directions.BACKWARD
                    }[self.direction]
        else:
            first_node_position, second_node_position = first.get_position(), second.get_position()
            if first_node_position[Position.X] == second_node_position[Position.X] and second_node_position[Position.X] == current_node.get_position()[Position.X]:
                if (first_node_position[Position.Y] > second_node_position[Position.Y] > current_node.get_position()[Position.Y]) or \
                        (first_node_position[Position.Y] < second_node_position[Position.Y] < current_node.get_position()[Position.Y]):
                    return Directions.FORWARD
                else:
                    return Directions.BACKWARD
            elif first_node_position[Position.Y] == second_node_position[Position.Y] and second_node_position[Position.Y] == current_node.get_position()[Position.Y]:
                if (first_node_position[Position.X] > second_node_position[Position.X] > current_node.get_position()[Position.X]) or \
                        (first_node_position[Position.X] < second_node_position[Position.X] < current_node.get_position()[Position.X]):
                    return Directions.FORWARD
                else:
                    return Directions.BACKWARD
            elif first_node_position[Position.X] == second_node_position[Position.X]:
                if first_node_position[Position.Y] < second_node_position[Position.Y]:
                    if second_node_position[Position.X] < current_node.get_position()[Position.X]:
                        return Directions.LEFT
                    else:
                        return Directions.RIGHT
                else:
                    if second_node_position[Position.X] > current_node.get_position()[Position.X]:
                        return Directions.LEFT
                    else:
                        return Directions.RIGHT
            else:
                if first_node_position[Position.X] < second_node_position[Position.X]:
                    return {
                        second_node_position[Position.Y] > current_node.get_position()[Position.Y]: Directions.LEFT,
                        second_node_position[Position.Y] < current_node.get_position()[Position.Y]: Directions.RIGHT
                    }[True]
                else:
                    return {
                        second_node_position[Position.Y] < current_node.get_position()[Position.Y]: Directions.LEFT,
                        second_node_position[Position.Y] > current_node.get_position()[Position.Y]: Directions.RIGHT
                    }[True]

        return -2

    def get_path(self, node):
        # Generates the path by backtracking from the final node to the start
        path = [self.go_where(node)]
        current_node = node.get_parent()
        if current_node is None:
            return None
        while current_node.get_parent() is not None:
            if self.go_where(current_node) >= 0:
                temp_path = [0] * (len(path) + 1)
                temp_path[1:] = path
                temp_path[0] = self.go_where(current_node)
                path = temp_path
                current_node = current_node.get_parent()
        return path

    def set_direction(self, direction):
        self.direction = direction

    def set_first(self, first):
        self.first = first

    def set_first_turn_penalty(self, first_penalty):
        self.first_penalty = first_penalty