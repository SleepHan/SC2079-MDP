import networkx as nx
import numpy as np
import itertools

class AStar:
    def __init__(self, dimension, turnRad, obstacles):
        self.dimX = dimension[0]
        self.dimY = dimension[1]
        self.turnRad = turnRad
        self.turnWeight = (np.pi/2) * turnRad
        self.G = nx.DiGraph()
        self.obstacles = [(x*10, y*10) for x, y, _ in obstacles]

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
            weights = [1, 1, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
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
            weights = [1, 1, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
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
            weights = [1, 1, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
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
            weights = [1, 1, self.turnWeight, self.turnWeight, self.turnWeight, self.turnWeight]
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
                if (x >= ox - 10 and x <= ox + 10) and (y >= oy - 10 and y <= oy + 10):
                    remList.append(node)

            if x < 10 or y < 10 or x >= 190 or y >= 190:
                remList.append(node)

        self.G.remove_nodes_from(remList)


    def pairwise(iterable):
        # pairwise('ABCDEFG') --> AB BC CD DE EF FG
        a, b = itertools.tee(iterable)
        next(b, None)
        return zip(a, b)
    

    # Getting shortest path using A* from networkx
    def search(self, start, end):
        startNode = (start[0]*10, start[1]*10, start[2])
        endNode = (end[0]*10, end[1]*10, end[2])

        # Return inf if no path was found
        try:
            path = nx.astar_path(self.G, startNode, endNode)
        except nx.NetworkXNoPath:
            return (float('inf'), None, None)

        movCmd = []
        totalDist = 0
        for src, dst in AStar.pairwise(path):
            mv = self.G.get_edge_data(src, dst)['mov']
            if mv in ['W', 'S']: totalDist += 1
            else: totalDist += self.turnWeight

            movCmd.append(mv)

        return (totalDist, movCmd, path)