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
finalPositions = [(2, 10, directions['East']),
                  (2, 10, directions['West'])]


startNode = 0
endNode = 0

class FastestCar:
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


    # Expected input for obs = (x, y, orien)
    def addObstacle(self, obs):
        self.positions.append(self.expectedPos(obs))


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
        print(permutation)
        print(distance)
        return (permutation, distance)


    # Displays paths selected to complete Hamiltonian cycle
    def displayPath(self, seq, num):
        index = [(i, (i+1)%num) for i in range(num)]
        rx, ry = [], []
        wx, wy = [], []
        sx, sy = [], []

        plt.grid(True)
        plt.axis('equal')
        plt.xticks(np.arange(0, 21, 1.0))
        plt.yticks(np.arange(0, 21, 1.0))

        for i in range(20):
            wx.append(i)
            wy.append(0)
        for i in range(20):
            wx.append(0)
            wy.append(i)
        for i in range(20):
            wx.append(i)
            wy.append(19)
        for i in range(20):
            wx.append(19)
            wy.append(i)

        plt.plot(wx, wy, '-k')


        for i in range(5):
            sx.append(i)
            sy.append(0)
        for i in range(5):
            sx.append(0)
            sy.append(i)
        for i in range(5):
            sx.append(i)
            sy.append(4)
        for i in range(5):
            sx.append(4)
            sy.append(i)

        plt.plot(sx, sy, '-g')


        for obstacle in self.obstacleList:
            ox, oy = obstacle[:2]
            if (obstacle[2] == 'N'): plt.plot(ox, oy, '^b')
            elif (obstacle[2] == 'S'): plt.plot(ox, oy, 'vb')
            elif (obstacle[2] == 'E'): plt.plot(ox, oy, '>b')
            else: plt.plot(ox, oy, '<b')


        for start, end in index:
            startNode = seq[start]
            endNode = seq[end]
            orien = self.positions[endNode][2]
            pathing = self.dubinsPath[startNode][endNode][2]

            for x,y in pathing:
                rx.append(x)
                ry.append(y)
            
                plt.plot(rx, ry, '-r')
                plt.pause(0.0005)

            # Print cross and arrow only at obstacles
            if not (x <=4 and y <=4):
                plt.plot(x, y, 'xk')
                if orien == directions['North']: plt.arrow(x, y, 0, 1, length_includes_head=True, head_width=0.5, head_length=0.5)
                elif orien == directions['South']: plt.arrow(x, y, 0, -1, length_includes_head=True, head_width=0.5, head_length=0.5)
                elif orien == directions['East']: plt.arrow(x, y, 1, 0, length_includes_head=True, head_width=0.5, head_length=0.5)
                else: plt.arrow(x, y, -1, 0, length_includes_head=True, head_width=0.5, head_length=0.5)
                
        plt.show()


    # Gives the details of each path selected for the Hamiltonian cycle
    def printPathInfo(self, seq, num):
        index = [(i, (i+1)%num) for i in range(num)]

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
        self.obstacleList = [(15, 10, 'W'),
                             (8, 10, 'W')]

        self.initPosition = (2, 10, directions['East'])

        # First element represents the inital position the RC will be in
        self.positions.append(self.initPosition)

        for obs in self.obstacleList:
            self.addObstacle(obs)

        print(self.positions)

        self.map = GridMap([20, 20])
        self.map.setOrigin(self.initPosition)
        for ob in self.obstacleList:
            self.map.setObstacles(ob)

