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

from heapq import heappop, heappush

def heuristic(position, goal):
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])

def find_shortest_path_a_star(maze, start, goal):
    rows, cols = len(maze), len(maze[0])

    # Initialize priority queue for A* search
    open_set = [(heuristic(start, goal), start)]
    closed_set = set()
    came_from = {}

    while open_set:
        _, current_position = heappop(open_set)

        if current_position == goal:
            path = reconstruct_path(came_from, start, goal)
            return path

        closed_set.add(current_position)

        # Add neighbors to the priority queue
        neighbors = [
            (current_position[0] - 1, current_position[1]),
            (current_position[0] + 1, current_position[1]),
            (current_position[0], current_position[1] - 1),
            (current_position[0], current_position[1] + 1)
        ]

        for neighbor in neighbors:
            if (
                0 <= neighbor[0] < rows
                and 0 <= neighbor[1] < cols
                and maze[neighbor[0]][neighbor[1]] != "E"
                and neighbor not in closed_set
            ):
                cost = heuristic(neighbor, goal)
                heappush(open_set, (cost, neighbor))
                came_from[neighbor] = current_position

    print("No path found.")
    return None

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []

    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()
    return path

def mark_path_on_maze(maze, path):
    marked_maze = [row.copy() for row in maze]

    for position in path:
        row, col = position
        if marked_maze[row][col] != "S" and marked_maze[row][col] != "x":
            marked_maze[row][col] = "*"

    return marked_maze

# Example maze
maze_a_star = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ["S", 0, 0, "x", 0, 0, 0, "x", 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ["E", 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

# Find the start and end positions
start_position = None
end_position = None
for i in range(len(maze_a_star)):
    for j in range(len(maze_a_star[0])):
        if maze_a_star[i][j] == "S":
            start_position = (i, j)
        elif maze_a_star[i][j] == "E":
            end_position = (i, j)

if not start_position or not end_position:
    print("Start or end position not found.")
else:
    obstacles = [(i, j) for i in range(len(maze_a_star)) for j in range(len(maze_a_star[0])) if maze_a_star[i][j] == "x"]

    # Visit obstacles in the shortest order
    total_path = [start_position]
    for obstacle in obstacles:
        shortest_path_to_obstacle = find_shortest_path_a_star(maze_a_star, total_path[-1], obstacle)
        if shortest_path_to_obstacle:
            total_path.extend(shortest_path_to_obstacle)

    # Return to the end position
    final_path_to_end = find_shortest_path_a_star(maze_a_star, total_path[-1], end_position)
    if final_path_to_end:
        total_path.extend(final_path_to_end)

    # Mark the path on the maze
    marked_maze = mark_path_on_maze(maze_a_star, total_path)

    # Print the marked maze
    for row in marked_maze:
        print(row)
