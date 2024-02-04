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

class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []
    open_list.append(start_node)

    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if (
                node_position[0] > (len(maze) - 1)
                or node_position[0] < 0
                or node_position[1] > (len(maze[len(maze) - 1]) - 1)
                or node_position[1] < 0
            ):
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                (child.position[1] - end_node.position[1]) ** 2
            )
            child.f = child.g + child.h

            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            open_list.append(child)


def visualize_path(maze, path):
    for row in range(len(maze)):
        for col in range(len(maze[row])):
            if (row, col) == path[0]:
                print("S", end=" ")  # Start
            elif (row, col) == path[-1]:
                print("E", end=" ")  # End
            elif (row, col) in path:
                print("*", end=" ")  # Path
            elif maze[row][col] == "N":
                print("n", end=" ")  # North obstacle
            elif maze[row][col] == "S":
                print("d", end=" ")  # South obstacle
            elif maze[row][col] == "W":
                print("w", end=" ")  # West obstacle
            elif maze[row][col] == "E":
                print("e", end=" ")  # East obstacle
            else:
                print(".", end=" ")  # Open space
        print()


def main():
    maze = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, "N", 0, 0, 0, "S", 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]

    start = (5, 0)
    end = (5, 9)

    path = astar(maze, start, end)
    visualize_path(maze, path)


if __name__ == "__main__":
    main()
