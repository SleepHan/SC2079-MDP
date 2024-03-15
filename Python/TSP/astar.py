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
    offV        - Vertical displacement of turn
    offH        - Horizontal displacement of turn
    corV        - Vertical displacement correction to round off
    corH        - Horizontal displacement correction to round off
    obstacles   - List of obstacles on the map
    '''
    def __init__(self, dimension, step, offV, offH, corV, corH, obstacles):
        self.dimX = dimension[0]
        self.dimY = dimension[1]
        # self.turnRad = turnRad
        self.step = step
        self.offV = offV
        self.offH = offH
        self.corV = corV
        self.corH = corH
        self.turnWeight = calcTurnWeight(offV*step, offH*step)
        self.G = nx.DiGraph()
        self.obstacles = [(x, y) for x, y, _ in obstacles]

        self.getFriends()
        self.removeInvalid()


    # Getting all possible future nodes for each grid
    def getFriends(self):
        weights = [self.step, self.step, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
        moves = ['W', 'T', 'A', 'D', 'F', 'H']

        for x, y in np.stack(np.meshgrid(np.arange(self.dimX), np.arange(self.dimY))).reshape((2, -1)).T:
            src =     (x, y, 'N')
            forwards =  (x, y+1, 'N')
            reverse =   (x, y-1, 'N')
            forwardsL = (x-self.offH, y+self.offV, 'W')
            forwardsR = (x+self.offH, y+self.offV, 'E')
            reverseL =  (x-self.offH, y-self.offV, 'E')
            reverseR =  (x+self.offH, y-self.offV, 'W')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])

            src =     (x, y, 'S')
            forwards =  (x, y-1, 'S')
            reverse =   (x, y+1, 'S')
            forwardsL = (x+self.offH, y-self.offV, 'E')
            forwardsR = (x-self.offH, y-self.offV, 'W')
            reverseL =  (x+self.offH, y+self.offV, 'W')
            reverseR =  (x-self.offH, y+self.offV, 'E')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])

            src =      (x, y, 'E')
            forwards =  (x+1, y, 'E')
            reverse =   (x-1, y, 'E')
            forwardsL = (x+self.offV, y+self.offH, 'N')
            forwardsR = (x+self.offV, y-self.offH, 'S')
            reverseL =  (x-self.offV, y+self.offH, 'S')
            reverseR =  (x-self.offV, y-self.offH, 'N')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])

            src =     (x, y, 'W')
            forwards =  (x-1, y, 'W')
            reverse =   (x+1, y, 'W')
            forwardsL = (x-self.offV, y-self.offH, 'S')
            forwardsR = (x-self.offV, y+self.offH, 'N')
            reverseL =  (x+self.offV, y-self.offH, 'N')
            reverseR =  (x+self.offV, y+self.offH, 'S')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])


    # Removing any cells that should not be accessible (Obstacle/Boundary)
    def removeInvalid(self):
        remList = []
        for node in self.G.nodes:
            x, y, facing = node
            # 40x40 grid (Each grid is 5x5)
            if self.step == 5:
                for ox, oy in self.obstacles:
                    if (x >= ox - 2 and x <= ox + 3) and (y >= oy - 2 and y <= oy + 3):
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
    def search(self, start, end):
        # Return inf if no path was found
        try:
            path = nx.astar_path(self.G, start, end)
        except (nx.NetworkXNoPath, nx.NodeNotFound) as error:
            # print(error)
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

                # Check for vertical adjustment
                if self.corV > 0:
                    if mv in ['A', 'D']:
                        movCmd.append(
                            {
                                'type':     'W',
                                'start':    src,
                                'length':   int(self.corV*self.step)
                            }
                        )
                    else:
                        movCmd.append(
                            {
                                'type':     'T',
                                'start':    src,
                                'length':   int(self.corV*self.step)
                            }
                        )
                
                # Start of turn movement
                movCmd.append(
                    {
                        'type':     mv,
                        'start':    src,
                        'angle':    90,
                        'length':   self.turnWeight
                    }
                )

                # Check for horizontal adjustment
                if self.corH > 0:
                    if mv in ['A', 'D']:
                        movCmd.append(
                            {
                                'type':     'W',
                                'end':      dst,
                                'length':   int(self.corH*self.step)
                            }
                        )
                    else:
                        movCmd.append(
                            {
                                'type':     'T',
                                'end':      dst,
                                'length':   int(self.corH*self.step)
                            }
                        )
                else:
                    movCmd[-1]['end'] = dst
                
                totalDist += self.turnWeight

        if straightType is not None:
            movCmd.append(
                {
                    'type':     straightType,
                    'start':    straightStart,
                    'end':      straightEnd,
                    'length':   straightDist
                }
            )

        return (totalDist, movCmd)