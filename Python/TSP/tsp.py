from dubins import Dubins
from astar import AStar

import matplotlib.pyplot as plt
import numpy as np
from python_tsp.exact import solve_tsp_dynamic_programming
import math


# Directions to radians
directions = {
    'N': np.pi/2,
    'E': 2*np.pi,
    'S': (3*np.pi)/2,
    'W': np.pi
}

# Expected orientation of robot when facing obstacle
robotPositions = {
    'N': 'S',
    'E': 'W',
    'S': 'N',
    'W': 'E'
}

startNode = 0
endNode = 0

class TSP:
    def __init__(self, initPosition, dimX, dimY, turnRad, distCalType):
        self.dimensions = (dimX, dimY)
        self.obstacleList = []
        self.initPositionRad = (initPosition[0], initPosition[1], directions[initPosition[2]])
        self.initPositionDir = (initPosition[0], initPosition[1], initPosition[2])
        self.positionsRad = [self.initPositionRad]
        self.positionsDir = [self.initPositionDir]
        self.dubinsPath = []
        self.dubinsDist = []
        self.aStarPath = []
        self.aStarDist = []
        self.hybridPath = []
        self.hybridDist = []
        self.turnRad = turnRad
        self.distCalType = distCalType


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
        finalPosRad = (ax, ay, directions[orient])
        finalPosDir = (ax, ay, orient)
        return finalPosRad, finalPosDir


    # Calculate the position of RC after backward movement
    def backward(self, pos):
        ax, ay, robOrient = pos
        backVal = 3

        if robOrient == directions['N'] or robOrient == 'N': ay -= backVal
        elif robOrient == directions['E'] or robOrient == 'E': ax -= backVal
        elif robOrient == directions['S'] or robOrient == 'S': ay += backVal
        else: ax += backVal

        return (ax, ay, robOrient)


    # Expected input for obs = (x, y, orien)
    def addObstacle(self, obs):
        self.obstacleList.append(obs)
        posRad, posDir = self.expectedPos(obs)
        self.positionsRad.append(posRad)
        self.positionsDir.append(posDir)


    # Calculates the shortest dubins path between each node
    # Returns False if there is an obstacle impossible to traverse to
    # Distance is calculated based on 20x20
    def calcDubins(self, step):
        print('Running Dubins')
        local_planner = Dubins(self.turnRad, step, self.obstacleList)

        for start in self.positionsRad:
            origin = start
            paths = []
    
            # If not initial position, assume that RC will reverse by 2 grids before moving
            if start != self.initPositionRad:
                start = self.backward(start)

            for dst in self.positionsRad:
                if dst == self.initPositionRad or origin == dst:
                    paths.append((float('inf'), None, None, None))

                else:
                    # Tuple: (Total_Dist, Angles/StrDist, PathCoordiates, SegmentConfig)
                    pathing = local_planner.dubins_path(start, dst)
                    paths.append(pathing)

            if origin != self.initPositionRad:
                minDist = [float('inf')]
                for x in range(1, 2):
                    for y in range(1, 2):
                        for _, angle in directions.items():
                            pathing = local_planner.dubins_path(start, (x, y, angle))
                            if pathing[0] < minDist[0]: minDist = pathing

                paths[0] = minDist

            self.dubinsPath.append(paths)

        self.dubinsDist = [[path[0] for path in node] for node in self.dubinsPath]

        if any(all(x == float('inf') for x in paths) for paths in self.dubinsDist):
            return False

        return True


    # Calculates the shortest path using A*
    # Distance is calculated based on 200x200
    def calAStar(self):
        print('Running A*')
        astarPlanner = AStar(self.dimensions, 10, self.turnRad, self.obstacleList)

        for start in self.positionsDir:
            origin = start
            paths = []

            if start != self.initPositionDir:
                start = self.backward(start)

            for dst in self.positionsDir:
                if dst == self.initPositionDir or origin == dst:
                    paths.append((float('inf'), None, None))
                else:
                    # Tuple: (Total_Dist, Movement_Command, Nodes_with_Orientation)
                    pathing = astarPlanner.search(start, dst)
                    paths.append(pathing)

            # Return back to start
            if origin != self.initPositionDir:
                minDist = (float('inf'), None, None)
                for x in range(1, 2):
                    for y in range(1, 2):
                        for key in directions.keys():
                            pathing = astarPlanner.search(start, (x, y, key))
                            if pathing[0] < minDist[0]: minDist = pathing

                paths[0] = minDist

            self.aStarPath.append(paths)

        self.aStarDist = [[path[0] for path in node] for node in self.aStarPath]

        if any(all(x == float('inf') for x in paths) for paths in self.aStarDist):
            return False

        return True


    # Makes use of the TSP DP library to get the sequence of obstacles to visit
    # Distance will be infinity if unable to find a cycle
    def calcTSP(self):
        if self.distCalType == 1:
            distance_matrix = np.matrix(self.dubinsDist)
        elif self.distCalType == 2:
            distance_matrix = np.matrix(self.aStarDist)

        permutation, distance = solve_tsp_dynamic_programming(distance_matrix)

        # distance_matrix = np.array([
        #     [0,  5, 4, 10],
        #     [5,  0, 8,  5],
        #     [4,  8, 0,  3],
        #     [10, 5, 3,  0]
        # ])
        # permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
        return (permutation, distance)


    # Displays paths selected to complete Hamiltonian cycle (Simulation)
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
            orien = self.positionsRad[endNode][2]
            pathing = self.dubinsPath[startNode][endNode][2]

            for seg in pathing:
                print('New Segment')
                print(seg)
                print()

                for x,y in seg:
                    rx.append(x)
                    ry.append(y)
                
                    plt.plot(rx, ry, '-r')
                    plt.pause(0.005)

                # Print cross and arrow only at obstacles
                if not (x <=4 and y <=4):
                    plt.plot(x, y, 'xk')
                    if orien == directions['N']: plt.arrow(x, y, 0, 1, length_includes_head=True, head_width=0.5, head_length=0.5)
                    elif orien == directions['S']: plt.arrow(x, y, 0, -1, length_includes_head=True, head_width=0.5, head_length=0.5)
                    elif orien == directions['E']: plt.arrow(x, y, 1, 0, length_includes_head=True, head_width=0.5, head_length=0.5)
                    else: plt.arrow(x, y, -1, 0, length_includes_head=True, head_width=0.5, head_length=0.5)
                
        plt.show()


    # Gives the details of each path selected for the Hamiltonian cycle
    # Generates the commands to send to STM
    # Gives the start and end coordinates for the Android side (Unrounded)
    def generateCommands(self, seq, num):
        index = [(i, (i+1)%num) for i in range(num)]
        fullPath = []
        currentPos = None

        # Getting path from start to end
        for start, end in index:
            print('===============New Path===============')
            segment = []
            segStart = None
            segEnd = None

            # Getting node index and path info
            startNode = seq[start]
            endNode = seq[end]
            dist, dubPath, pathPts, config = self.dubinsPath[startNode][endNode]

            zipped = [(config[0], dubPath[0], pathPts[0]), (config[1], dubPath[2], pathPts[1]), (config[2], dubPath[1], pathPts[2])]
            print('{} -> {}'.format(self.positionsRad[startNode] if currentPos is None else currentPos, self.positionsRad[endNode]))

            if currentPos is not None:
                print('Back: 2')
                print('Actual Length: 20\n')
                segment.append(['B', 2, 20, currentPos, pathPts[0][0]])

            # Going through each segment of path
            for segType, length, pathCoor in zipped:
                startCoor = np.rint(pathCoor[0])
                endCoor = np.rint(pathCoor[-1])
                
                if segStart is None: segStart = startCoor if currentPos is None else currentPos

                if segType == 'l':
                    print('Left: {} rad, {}'.format(length, self.turnRad*(abs(length))))
                    print('Actual Length: {}\n'.format(self.turnRad*abs(length)*10))
                    segment.append(['L', math.degrees(abs(length)), self.turnRad*(abs(length))*10, startCoor, endCoor])
                elif segType == 'r':
                    print('Right: {} rad, {}'.format(length, self.turnRad*(abs(length))))
                    print('Actual Length: {}\n'.format(self.turnRad*abs(length)*10))
                    segment.append(['R', math.degrees(abs(length)), self.turnRad*(abs(length))*10, startCoor, endCoor])
                else:
                    print('Straight: {}'.format(length))
                    print('Actual Length: {}\n'.format(length*10))
                    segment.append(['S', length, length*10, startCoor, endCoor])

            print('\nFull Distance; {}\n'.format(dist))

            currentPos = endCoor
            segEnd = endCoor
            obstacle = self.obstacleList[endNode-1] if endNode != 0 else segEnd
            fullPath.append([segment, segStart, segEnd, obstacle])
        
        return fullPath


    # Prints the desired info
    def printInfo(self, num):
        if num == 1:
            for node in self.dubinsPath:
                print(node)

        elif num == 2:
            print('Distance Matrix')
            print(np.matrix(self.distance))

        print()


    # Dummy values for testing purposes 
    def testObstacles(self, choice=0):
        if choice == 1:
            obstacles = [(2, 17, 'S'), (17, 13, 'W'), (14, 4, 'N'), (8, 5, 'E'), (5, 12, 'N')]
        elif choice == 2:
            obstacles = [(2, 18, 'S'), (6, 12, 'N'), (8, 5, 'E'), (15, 16, 'S'), (16, 1, 'W')]
        elif choice == 3:
            obstacles = [(1, 18, 'S'), (6, 12, 'N'), (10, 7, 'E'), (15, 16, 'S'), (19, 9, 'W'), (13, 2, 'W')]
        elif choice == 4:
            obstacles = [(1, 14, 'E'), (5, 12, 'S'), (8, 5, 'N'), (11, 14, 'E'), (15, 2, 'W'), (16, 19, 'S'), (19, 9, 'W')]
        elif choice == 5:
            obstacles = [(1, 14, 'E')]
        elif choice == 6:
            obstacles = [(10, 10, 'S')]
        else:
            obstacles = [(10, 10, 'N'), (10, 10, 'S'), (10, 10, 'E'), (10, 10, 'W')]

        for obs in obstacles:
            self.addObstacle(obs)


    # Main driver function
    def run(self):
        res = False
        if self.distCalType == 1:
            res = self.calcDubins(1)
        elif self.distCalType == 2:
            res = self.calAStar()

        if res: sequence, finalDist = self.calcTSP()
        else: print('No valid path available')

        if finalDist == float('inf'): print('No valid path available')
        else:
            print(finalDist)
            print(sequence)