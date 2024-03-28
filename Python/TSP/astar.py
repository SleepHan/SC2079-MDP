import networkx as nx
import numpy as np
import itertools

def calcTurnWeight(offV, offH):
    # Makes use of Ramanujan Approximation to get arc length of ellipse
    # Seems to work just as well for circles as well
    t = ((offV - offH) / (offV + offH)) ** 2
    perimeter = np.pi * (offV + offH) * (1 + (3*t / (10 + np.sqrt(4 - 3*t))))
    return  perimeter / 4


class AStar:
    '''
    dimension   - (Length of x-axis, Length of y-axis)
    step        - Length of each grid (Same for x- and y-axis)
    offFV       - Forward vertical displacement of turn
    offFH       - Forward horizontal displacement of turn
    offRV       - Reverse vertical displacement of turn
    offRH       - Reverse horizontal displacement of turn
    obstacles   - List of obstacles on the map
    '''
    def __init__(self, dimension, step, offFV, offFH, offRV, offRH, obstacles):
        self.dimX = dimension[0]
        self.dimY = dimension[1]
        self.step = step
        self.offFV = offFV
        self.offFH = offFH
        self.offRV = offRV
        self.offRH = offRH
        self.fwdTurnWeight = calcTurnWeight(offFV*step, offFH*step)
        self.bwdTurnWeight = calcTurnWeight(offRV*step, offRH*step)
        self.G = nx.DiGraph()
        self.obstacles = [(x, y) for x, y, _ in obstacles]
        self.invalid = self.getInvalidNodes()

        self.getFriends()
        self.removeInvalid()


    # Getting nodes that are not traversable
    def getInvalidNodes(self):
        invalid = []
        for obs in self.obstacles:
            x, y = obs
            invalid += [(x+offX, y+offY) for offX in range(-1, 1) for offY in range(-1, 1)]

        return invalid


    # Checking if move is valid
    def validMove(self, node, xMove, yMove):
        # Num of cells moved = xMove + yMove
        # N/S -> Update Y-coor -> Update X-coor
        # E/W -> Update X-coor -> Update Y-coor
        x, y, face = node
        traversed = [(x, y)]
        if face in ['N', 'S']:
            if yMove > 0:
                for offY in range(1, yMove+1):
                    traversed.append((x, y+offY))
            else:
                for offY in range(0, yMove-1, -1):
                    traversed.append((x, y+offY))

            if xMove > 0:
                for offX in range(1, xMove+1):
                    traversed.append((x+offX, y+offY))
            else:
                for offX in range(0, xMove-1, -1):
                    traversed.append((x+offX, y+offY))
        else:
            if xMove > 0:
                for offX in range(1, xMove+1):
                    traversed.append((x+offX, y))
            else:
                for offX in range(0, xMove-1, -1):
                    traversed.append((x+offX, y))

            if yMove > 0:
                for offY in range(1, yMove+1):
                    traversed.append((x+offX, y+offY))
            else:
                for offY in range(0, yMove-1, -1):
                    traversed.append((x+offX, y+offY))
        
        # if any(bad in traversed for bad in self.invalid):
        #     print('INVALID')
        #     return False
        # return True

        return not any(bad in traversed for bad in self.invalid)


    # Getting all possible future nodes for each grid
    def getFriends(self):
        for x, y in np.stack(np.meshgrid(np.arange(self.dimX), np.arange(self.dimY))).reshape((2, -1)).T:
            # Starting North
            src =     (x, y, 'N')
            self.G.add_node(src)

            forwards =  (x, y+1, 'N')
            self.G.add_node(forwards)
            self.G.add_edge(src, forwards, weight=self.step, mov='W')

            reverse =   (x, y-1, 'N')
            self.G.add_node(reverse)
            self.G.add_edge(src, reverse, weight=self.step, mov='T')

            if (self.validMove(src, -self.offFH, self.offFV)):
                forwardsL = (x-self.offFH, y+self.offFV, 'W')
                self.G.add_node(forwardsL)
                self.G.add_edge(src, forwardsL, weight=self.fwdTurnWeight, mov='A')
            
            if (self.validMove(src, self.offFH, self.offFV)):
                forwardsR = (x+self.offFH, y+self.offFV, 'E')
                self.G.add_node(forwardsR)
                self.G.add_edge(src, forwardsR, weight=self.fwdTurnWeight, mov='D')

            if (self.validMove(src, -self.offRH, -self.offRV)):
                reverseL =  (x-self.offRH, y-self.offRV, 'E')
                self.G.add_node(reverseL)
                self.G.add_edge(src, reverseL, weight=self.bwdTurnWeight, mov='F')

            if (self.validMove(src, self.offRH, -self.offRV)):
                reverseR =  (x+self.offRH, y-self.offRV, 'W')
                self.G.add_node(reverseR)
                self.G.add_edge(src, reverseR, weight=self.bwdTurnWeight, mov='H')


            # Starting South
            src =     (x, y, 'S')
            self.G.add_node(src)

            forwards =  (x, y-1, 'S')
            self.G.add_node(forwards)
            self.G.add_edge(src, forwards, weight=self.step, mov='W')

            reverse =   (x, y+1, 'S')
            self.G.add_node(reverse)
            self.G.add_edge(src, reverse, weight=self.step, mov='T')

            if (self.validMove(src, self.offFH, -self.offFV)):
                forwardsL = (x+self.offFH, y-self.offFV, 'E')
                self.G.add_node(forwardsL)
                self.G.add_edge(src, forwardsL, weight=self.fwdTurnWeight, mov='A')
            
            if (self.validMove(src, -self.offFH, -self.offFV)):
                forwardsR = (x-self.offFH, y-self.offFV, 'W')
                self.G.add_node(forwardsR)
                self.G.add_edge(src, forwardsR, weight=self.fwdTurnWeight, mov='D')

            if (self.validMove(src, self.offRH, self.offRV)):
                reverseL =  (x+self.offRH, y+self.offRV, 'W')
                self.G.add_node(reverseL)
                self.G.add_edge(src, reverseL, weight=self.bwdTurnWeight, mov='F')

            if (self.validMove(src, -self.offRH, self.offRV)):
                reverseR =  (x-self.offRH, y+self.offRV, 'E')
                self.G.add_node(reverseR)
                self.G.add_edge(src, reverseR, weight=self.bwdTurnWeight, mov='H')


            # Starting East
            src =      (x, y, 'E')
            self.G.add_node(src)

            forwards =  (x+1, y, 'E')
            self.G.add_node(forwards)
            self.G.add_edge(src, forwards, weight=self.step, mov='W')

            reverse =   (x-1, y, 'E')
            self.G.add_node(reverse)
            self.G.add_edge(src, reverse, weight=self.step, mov='T')

            if (self.validMove(src, self.offFH, self.offFV)):
                forwardsL = (x+self.offFV, y+self.offFH, 'N')
                self.G.add_node(forwardsL)
                self.G.add_edge(src, forwardsL, weight=self.fwdTurnWeight, mov='A')
            
            if (self.validMove(src, self.offFH, -self.offFV)):
                forwardsR = (x+self.offFV, y-self.offFH, 'S')
                self.G.add_node(forwardsR)
                self.G.add_edge(src, forwardsR, weight=self.fwdTurnWeight, mov='D')

            if (self.validMove(src, -self.offRH, self.offRV)):
                reverseL =  (x-self.offRV, y+self.offRH, 'S')
                self.G.add_node(reverseL)
                self.G.add_edge(src, reverseL, weight=self.bwdTurnWeight, mov='F')

            if (self.validMove(src, -self.offRH, -self.offRV)):
                reverseR =  (x-self.offRV, y-self.offRH, 'N')
                self.G.add_node(reverseR)
                self.G.add_edge(src, reverseR, weight=self.bwdTurnWeight, mov='H')

            
            # Starting West
            src =     (x, y, 'W')
            self.G.add_node(src)

            forwards =  (x-1, y, 'W')
            self.G.add_node(forwards)
            self.G.add_edge(src, forwards, weight=self.step, mov='W')

            reverse =   (x+1, y, 'W')
            self.G.add_node(reverse)
            self.G.add_edge(src, reverse, weight=self.step, mov='T')

            if (self.validMove(src, -self.offFH, -self.offFV)):
                forwardsL = (x-self.offFV, y-self.offFH, 'S')
                self.G.add_node(forwardsL)
                self.G.add_edge(src, forwardsL, weight=self.fwdTurnWeight, mov='A')
            
            if (self.validMove(src, -self.offFH, self.offFV)):
                forwardsR = (x-self.offFV, y+self.offFH, 'N')
                self.G.add_node(forwardsR)
                self.G.add_edge(src, forwardsR, weight=self.fwdTurnWeight, mov='D')

            if (self.validMove(src, self.offRH, -self.offRV)):
                reverseL =  (x+self.offRV, y-self.offRH, 'N')
                self.G.add_node(reverseL)
                self.G.add_edge(src, reverseL, weight=self.bwdTurnWeight, mov='F')

            if (self.validMove(src, self.offRH, self.offRV)):
                reverseR =  (x+self.offRV, y+self.offRH, 'S')
                self.G.add_node(reverseR)
                self.G.add_edge(src, reverseR, weight=self.bwdTurnWeight, mov='H')


    # Removing any cells that should not be accessible (Obstacle/Boundary)
    # Note that the coordinates of the obstacle represent the bottom left-corner of the obstacle
    def removeInvalid(self):
        remList = []
        for node in self.G.nodes:
            x, y, _ = node
            # 40x40 grid (Each grid is 5x5)
            if self.step == 5:
                for ox, oy in self.obstacles:
                    if (x >= ox - 2 and x <= ox + 4) and (y >= oy - 2 and y <= oy + 4):
                        remList.append(node)
                    
                if x < 2 or y < 2 or x > 37 or y > 37:
                    remList.append(node)

            # 20x20 grid (Each grid is 10x10)
            else:
                for ox, oy in self.obstacles:
                    if (x >= ox - 1 and x <= ox + 1) and (y >= oy - 1 and y <= oy + 1):
                        remList.append(node)

                if x < 1 or y < 1 or x > 18 or y > 18:
                    remList.append(node)


        self.G.remove_nodes_from(remList)


    def pairwise(iterable):
        # pairwise('ABCDEFG') --> AB BC CD DE EF FG
        a, b = itertools.tee(iterable)
        next(b, None)
        return zip(a, b)
    

    # Getting shortest path using A* from networkx
    def search(self, start, end, endBack=None):
        # Return inf if no path was found
        try:
            path = nx.astar_path(self.G, start, end)
        except nx.NetworkXNoPath as error:
            print('No path: ', error)
            return (float('inf'), None, None)
        except nx.NodeNotFound as error:
            print('Node removed: ', error)
            return (float('inf'), None, None)

        movCmd = []
        totalDist = 0
        straightType = None
        straightDist = 0
        straightStart = None
        straightEnd  = None
        for src, dst in AStar.pairwise(path):
            mv = self.G.get_edge_data(src, dst)['mov']
            if mv in ['W', 'T']: 
                # New straight movement
                if straightType != mv:
                    straightStart = src
                    straightType = mv

                straightEnd = dst
                straightDist += self.step
                totalDist += self.step
            else: 
                # End of straight movement
                if straightDist > 0:
                    movCmd.append(
                        {
                            'type':     straightType,
                            'start':    straightStart,
                            'end':      straightEnd,
                            'length':   straightDist
                        }
                    )
                    straightDist = 0
                    straightType = None
                    straightStart = None
                    straightEnd = None
                
                if mv in ['A', 'D']: turnWeight = self.fwdTurnWeight
                else: turnWeight = self.bwdTurnWeight

                # Start of turn movement
                movCmd.append(
                    {
                        'type':     mv,
                        'start':    src,
                        'end':      dst,
                        'angle':    90,
                        'length':   turnWeight
                    }
                )
                
                totalDist += turnWeight

        if straightType is not None:
            movCmd.append(
                {
                    'type':     straightType,
                    'start':    straightStart,
                    'end':      straightEnd,
                    'length':   straightDist
                }
            )

        if endBack is not None:
            movCmd.append(
                {
                    'type':     'T',
                    'start':    straightEnd if straightEnd is not None else dst,
                    'end':      endBack,
                    'length':   30
                }
            )

        return (totalDist, movCmd)