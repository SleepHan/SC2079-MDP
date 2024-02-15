import math

class GridMap:
    # Creates an empty map
    def __init__(self, dimensions):
        self.dimensions = dimensions
        self.obstacles = []
        

    # Sets obstacles on the map
    def setObstacles(self, obstacle):
        self.obstacles.append(obstacle)


    # Checks if pathing is a valid move
    def validMove(self, path):
        for point in path:
            ax, ay = point[:2]

            # Border Checking
            if ax < 1 or ax > 18: 
                print('X Collide: [{}][{}]'.format(ax, ay))
                return False
            if ay < 1 or ay > 18: 
                print('Y Collide: [{}][{}]'.format(ax, ay))
                return False
            
            # Obstacle Checking
            for obs in self.obstacles:
                ox, oy, _ = obs
                if ox - 1 <= ax <= ox + 1 and oy - 1 <= ay <= oy + 1:
                    print('Obstacle Collide: [{}][{}]'.format(ox, oy))
                    return False
        
        print('VALID')
        print()
        return True