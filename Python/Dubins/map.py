import math

class GridMap:
    # Creates an empty map
    def __init__(self, dimensions):
        self.dimensions = dimensions
        self.gridMap = [['.' for _ in range(dimensions[0])] for _ in range(dimensions[1])]
        self.obstacles = []

    # Sets obstacles on the map
    def setObstacles(self, obstacle):
        self.gridMap[obstacle[1]][obstacle[0]] = obstacle[2]
        self.updateWalls(obstacle)
        self.obstacles.append(obstacle)

    
    # Updates map with any new walls from added obstacle
    def updateWalls(self, obstacle):
        ax, ay = obstacle[:2]

        # Check if new obstacle near to boundary
        # X-axis
        if ax <= 2: self.setWall([[x, ay] for x in range(ax)])
        elif ax >= self.dimensions[0]-3: self.setWall([x, ay] for x in range(ax+1, self.dimensions[0]))

        # Y-axis
        if ay <= 2: self.setWall([ax, y] for y in range(ay))
        elif ay >= self.dimensions[1]-3: self.setWall([ax, y] for y in range(ay+1, self.dimensions[1]))


    # Sets wall on grid map
    def setWall(self, wall):
        for point in wall:
            ax , ay = point[:2]
            self.gridMap[ay][ax] = 'X'


    def validMove(self, path):
        for point in path:
            ax, ay = point[:2]
            ax = math.ceil(ax)
            ay = math.ceil(ay)

            if ax < 0 or ax > 19: return False
            if ay < 0 or ay > 19: return False
            if self.gridMap[ay][ax] != '.': return False
        
        return True
    
    def printGrid(self):
        for y in self.gridMap:
            print(y)