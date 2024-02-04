import math

class GridMap:
    # Creates an empty map
    def __init__(self, dimensions):
        self.dimensions = dimensions
        self.gridMap = [['.' for _ in range(dimensions[0])] for _ in range(dimensions[1])]
        self.obstacles = []

    # Sets obstacles on the map
    def setObstacles(self, obstacle):
        try:
            print(f"Setting obstacle: {obstacle}")
            self.gridMap[obstacle[1]][obstacle[0]] = obstacle[2]
        except IndexError as e:
            print(f"Error: {e}")
            print(f"Obstacle tuple causing the issue: {obstacle}")
        
        self.updateWalls(obstacle)
        self.obstacles.append(obstacle)
    
    

    # Updates map with any new walls from added obstacle
    def updateWalls(self, newObstacle):
        ax, ay = newObstacle[:2]

        # Check if new obstacle near to boundary
        # X-axis
        if ax <= 2:
            self.setWall([[x, ay] for x in range(ax + 1)])  # Corrected range
        elif ax >= self.dimensions[0] - 3:
            self.setWall([x, ay] for x in range(ax, self.dimensions[0]))  # Corrected range

        # Y-axis
        if ay <= 2:
            self.setWall([ax, y] for y in range(ay + 1))  # Corrected range
        elif ay >= self.dimensions[1] - 3:
            self.setWall([ax, y] for y in range(ay, self.dimensions[1]))  # Corrected range

        # Check if space in between obstacles is safe to move through
        for obstacle in self.obstacles:
            # X-axis
            if obstacle[0] == ax and abs(obstacle[1] - ay) < 3:
                if obstacle[1] > ay:
                    self.setWall([[ax, y] for y in range(ay + 1, obstacle[1])])  # Corrected range
                else:
                    self.setWall([[ax, y] for y in range(obstacle[1], ay)])  # Corrected range

            # Y-axis
            if obstacle[1] == ay and abs(obstacle[0] - ax) < 3:
                if obstacle[0] > ax:
                    self.setWall([[x, ay] for x in range(ax + 1, obstacle[0])])  # Corrected range
                else:
                    self.setWall([[x, ay] for x in range(obstacle[0], ax)])  # Corrected range

    

    # Sets wall on grid map
    def setWall(self, wall):
        for point in wall:
            ax, ay = point[:2]
            if 0 <= ax < self.dimensions[0] and 0 <= ay < self.dimensions[1]:
                self.gridMap[ay][ax] = 'X'


    # Checks if pathing is a valid move
    def validMove(self, path):
        for point in path:
            ax, ay = point[:2]
            ax = math.ceil(ax)
            ay = math.ceil(ay)

            if ax < 0 or ax > 19: return False
            if ay < 0 or ay > 19: return False
            if self.gridMap[ay][ax] != '.': return False
        
        return True
    
    def get_neighbors(self, position):
        x, y = position
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]  # Assuming 4-connected grid
        valid_neighbors = [(x, y) for x, y in neighbors if self.isValid(x, y)]
        return valid_neighbors
    
    # Checks if a given position is valid within the grid and not blocked by an obstacle
    def isValid(self, x, y):
        return 0 <= x < self.dimensions[0] and 0 <= y < self.dimensions[1] and self.gridMap[y][x] == '.'
    

    def printGrid(self):
        for y in self.gridMap[::-1]:
            print(y)