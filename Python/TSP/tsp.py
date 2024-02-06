from dubins import Dubins
from map import GridMap

import matplotlib.pyplot as plt
import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming


# Directions to radians
directions = {
    'North': np.pi/2,
    'East': 2*np.pi,
    'South': (3*np.pi)/2,
    'West': np.pi
}

# Expected orientation of robot when facing obstacle
robotPositions = {
    'N': directions['South'],
    'E': directions['West'],
    'S': directions['North'],
    'W': directions['East']
}

# Possible positions RC will complete its pathing in
finalPositions = [(1, 1, directions['South']),
                  (1, 2, directions['West'])]


class TSP:
    def __init__(self, initPosition):
        self.obstacleList = []
        self.initPosition = (initPosition[0], initPosition[1], directions[initPosition[2]])
        self.positions = []
        self.dubinsPath = []
        self.distance = []
        self.map = None


    # Positions RC is expected to be in to accurately capture the image of the obstacles
    # Obstacle Orientation to RC Position
    # North:    Y-Axis + 2, Directions to be South
    # East:     X-Axis + 2, Directions to be West
    # South:    Y-Axis - 2, Directions to be North
    # West:     X-Axis - 2, Directions to be East
    def expectedPos(self, obstacle):
        ax, ay, obsOrient = obstacle

        if obsOrient == 'N': ay += 2
        elif obsOrient == 'E': ax += 2
        elif obsOrient == 'S': ay -= 2
        else: ax -= 2
        
        orient = robotPositions[obsOrient]
        finalPos = (ax, ay, orient)
        return finalPos


    # Calculate the position of RC after backward movement
    def backward(self, pos):
        ax, ay, robOrient = pos
        backVal = 2

        if robOrient == directions['North']: ay -= backVal
        elif robOrient == directions['East']: ax -= backVal
        elif robOrient == directions['South']: ay += backVal
        else: ax += backVal

        return (ax, ay, robOrient)


    # Calculates the shortest dubins path between each node
    def calcDubins(self, turnRad, step):
        local_planner = Dubins(turnRad, step)

        for start in self.positions:
            origin = start
            paths = []
    
            # If not initial position, assume that RC will reverse by 2 grids before moving
            if start != self.initPosition:
                start = self.backward(start)

            for dst in self.positions:
                if dst == self.initPosition or origin == dst:
                    paths.append(float('inf'))

                else:
                    # Tuple: (Total_Dist, Specific_Dist, Pathing)
                    pathing = local_planner.dubins_path(start, dst, self.map)

                    # Distance stuff
                    # if not isinstance(pathing, float):
                    #     print(pathing[3])
                    #     print(pathing[1])

                    #     if pathing[3] == 'rlr' or pathing[3] == 'lrl':
                    #         print(2.5*(abs(pathing[1][0]) + abs(pathing[1][1]) + abs(pathing[1][2])))
                    #     else:
                    #         print(2.5*(abs(pathing[1][0]) + abs(pathing[1][1])) + pathing[1][2])
                    #     print(pathing[0])
                    
                    paths.append(pathing)

            if origin != self.initPosition:
                finalPos0 = local_planner.dubins_path(start, finalPositions[0], self.map)
                finalPos1 = local_planner.dubins_path(start, finalPositions[1], self.map)

                if isinstance(finalPos0, float): paths[0] = finalPos1
                elif isinstance(finalPos1, float): paths[0] = finalPos0
                else: paths[0] = finalPos0 if finalPos0[0] < finalPos1[0] else finalPos1

            self.dubinsPath.append(paths)

        self.distance = [[path if isinstance(path, float) else path[0] for path in node] for node in self.dubinsPath]


    def calcTSP(self):
        distance_matrix = np.matrix(self.distance)
        permutation, distance = solve_tsp_dynamic_programming(distance_matrix)

        # distance_matrix = np.array([
        #     [0,  5, 4, 10],
        #     [5,  0, 8,  5],
        #     [4,  8, 0,  3],
        #     [10, 5, 3,  0]
        # ])
        # permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
        return (permutation, distance)


    # Displays paths selected to complete Hamiltonian cycle
    def displayPath(self, seq, len):
        index = [(i, (i+1)%len) for i in range(len)]

        plt.grid()
        plt.xticks(np.arange(0, 21, 1.0))
        plt.yticks(np.arange(0, 21, 1.0))

        for start, end in index:
            startNode = seq[start]
            endNode = seq[end]
            pathing = self.dubinsPath[startNode][endNode][2]
            plt.plot(pathing[:, 0], pathing[:, -1])

        plt.show()


    # Gives the details of each path selected for the Hamiltonian cycle
    def printPathInfo(self, seq, len):
        index = [(i, (i+1)%len) for i in range(len)]

        for start, end in index:
            startNode = seq[start]
            endNode = seq[end]
            dubPath = self.dubinsPath[startNode][endNode][1]
            dist = self.dubinsPath[startNode][endNode][0]
            config = self.dubinsPath[startNode][endNode][3]

            zipped = [(config[0], dubPath[0]), (config[1], dubPath[2]), (config[2], dubPath[1])]
            print('{} -> {}'.format(self.positions[startNode], self.positions[endNode]))
            
            for segType, length in zipped:
                if segType == 'l':
                    print('Left: {} rad, {}'.format(length, 2.5*(abs(length))))
                elif segType == 'r':
                    print('Right: {} rad, {}'.format(length, 2.5*(abs(length))))
                else:
                    print('Straight: {}'.format(length))

            print('{}\n'.format(dist))


    # Prints the desired info
    def printInfo(self, num):
        if num == 1:
            for node in self.dubinsPath:
                print(node)

        elif num == 2:
            for node in self.distance:
                print(node)

        elif num == 3:
            self.map.printGrid()


    # Random values for testing purposes 
    def testObstacles(self):
        obstacleList = [(11, 10, 'S'),
                        (6, 7, 'E'),
                        (7, 15, 'S')]

        initPosition = (2, 2, directions['North'])

        # First element represents the inital position the RC will be in
        self.positions.append(initPosition)

        for obs in obstacleList:
            self.positions.append(self.expectedPos(obs))

        self.map = GridMap([20, 20])
        self.map.setOrigin(initPosition)
        for ob in obstacleList:
            self.map.setObstacles(ob)

