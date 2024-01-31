# https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
from map_test import GridMap
import math

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(grid_map, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] < 0 or node_position[0] >= grid_map.dimensions[0] or \
               node_position[1] < 0 or node_position[1] >= grid_map.dimensions[1]:
                continue

            # Make sure walkable terrain
            if not grid_map.validMove([node_position]):
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if any(child == closed_child for closed_child in closed_list):
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                        (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if any(child == open_node and child.g > open_node.g for open_node in open_list):
                continue

            # Add the child to the open list
            open_list.append(child)
            


def visualize_path(grid_map, path):
    """Visualize the path on the grid map"""
    if path is None:
        print("No path found.")
        return

    for row in range(grid_map.dimensions[1]):
        for col in range(grid_map.dimensions[0]):
            current_position = (col, row)
            if path and current_position == path[0]:
                print("S", end=" ")  # Start
            elif path and current_position == path[-1]:
                print("E", end=" ")  # End
            elif path and current_position in path:
                print("*", end=" ")  # Path
            elif not grid_map.validMove([current_position]):
                print("#", end=" ")  # Obstacle
            else:
                print(".", end=" ")  # Open space
        print()


def main():
    dimensions = (10, 10)

    grid_map = GridMap(dimensions)
    """Debugging"""
    # Before calling visualize_path in the main function
    grid_map.printGrid()

    obstacles = [(4, 0, 'X'), (4, 1, 'X'), (4, 2, 'X'), (4, 3, 'X'), (4, 4, 'X'),
                 (4, 6, 'X'), (4, 7, 'X'), (4, 8, 'X'), (4, 9, 'X')]

    for obstacle in obstacles:
        grid_map.setObstacles(obstacle)

    start = (0, 0)
    end = (7, 6)

    path = astar(grid_map, start, end)
    visualize_path(grid_map, path)


if __name__ == '__main__':
    main()
