import networkx as nx
import numpy as np
import itertools

class AStar:
    def __init__(self, dimension, step, turnRad, obstacles):
        self.dimX = dimension[0]
        self.dimY = dimension[1]
        self.turnRad = turnRad
        self.step = step
        self.turnWeight = (np.pi/2) * turnRad * 10
        self.G = nx.DiGraph()
        self.obstacles = [(x, y) for x, y, _ in obstacles]

        self.getFriends()
        self.removeInvalid()


    # Getting all possible future nodes for each grid
    def getFriends(self):
        for x, y in np.stack(np.meshgrid(np.arange(self.dimX), np.arange(self.dimY))).reshape((2, -1)).T:
            src =     (x, y, 'N')
            forwards =  (x, y+1, 'N')
            reverse =   (x, y-1, 'N')
            forwardsL = (x-self.turnRad, y+self.turnRad, 'W')
            forwardsR = (x+self.turnRad, y+self.turnRad, 'E')
            reverseL =  (x-self.turnRad, y-self.turnRad, 'E')
            reverseR =  (x+self.turnRad, y-self.turnRad, 'W')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            weights = [self.step, self.step, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
            moves = ['W', 'S', 'AW', 'DW', 'AS', 'DS']
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])

            src =     (x, y, 'S')
            forwards =  (x, y-1, 'S')
            reverse =   (x, y+1, 'S')
            forwardsL = (x+self.turnRad, y-self.turnRad, 'E')
            forwardsR = (x-self.turnRad, y-self.turnRad, 'W')
            reverseL =  (x+self.turnRad, y+self.turnRad, 'W')
            reverseR =  (x-self.turnRad, y+self.turnRad, 'E')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            weights = [self.step, self.step, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
            moves = ['W', 'S', 'AW', 'DW', 'AS', 'DS']
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])

            src =      (x, y, 'E')
            forwards =  (x+1, y, 'E')
            reverse =   (x-1, y, 'E')
            forwardsL = (x+self.turnRad, y+self.turnRad, 'N')
            forwardsR = (x+self.turnRad, y-self.turnRad, 'S')
            reverseL =  (x-self.turnRad, y+self.turnRad, 'S')
            reverseR =  (x-self.turnRad, y-self.turnRad, 'N')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            weights = [self.step, self.step, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
            moves = ['W', 'S', 'AW', 'DW', 'AS', 'DS']
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])

            src =     (x, y, 'W')
            forwards =  (x-1, y, 'W')
            reverse =   (x+1, y, 'W')
            forwardsL = (x-self.turnRad, y-self.turnRad, 'S')
            forwardsR = (x-self.turnRad, y+self.turnRad, 'N')
            reverseL =  (x+self.turnRad, y-self.turnRad, 'N')
            reverseR =  (x+self.turnRad, y+self.turnRad, 'S')

            friend_nodes = [forwards, reverse, forwardsL, forwardsR, reverseL, reverseR]
            weights = [self.step, self.step, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
            moves = ['W', 'S', 'AW', 'DW', 'AS', 'DS']
            friend_dicts = [dict(weight=weight, mov=mov) for weight, mov in zip(weights, moves)]

            self.G.add_node(src)
            self.G.add_nodes_from(friend_nodes)
            self.G.add_edges_from([(src, i, attrib_dict) for i, attrib_dict in zip(friend_nodes, friend_dicts)])


    # Removing any cells that should not be accessible (Obstacle/Boundary)
    def removeInvalid(self):
        remList = []
        for node in self.G.nodes:
            x, y, facing = node
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
        except nx.NetworkXNoPath:
            return (float('inf'), None, None)

        movCmd = []
        trimmedPath = [path[0]]
        totalDist = 0
        straightType = None
        straightDist = 0
        straightEnd  = None
        for src, dst in AStar.pairwise(path):
            mv = self.G.get_edge_data(src, dst)['mov']
            if mv in ['W', 'S']: 
                # New straight movement
                if straightType != mv: 
                    straightType = mv

                straightEnd = dst
                straightDist += self.step
                totalDist += self.step
            else: 
                # End of straight movement
                if straightDist > 0:
                    movCmd.append('{} {}'.format(straightType, straightDist))
                    trimmedPath.append(straightEnd)
                    straightDist = 0
                    straightType = None
                    straightEnd = None

                totalDist += self.turnWeight
                trimmedPath.append(dst)

                movCmd.append(mv)

        if straightType is not None:
            movCmd.append('{} {}'.format(straightType, straightDist))
            trimmedPath.append(straightEnd)

        return (totalDist, movCmd, trimmedPath)