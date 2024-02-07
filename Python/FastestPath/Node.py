# imports
class Node():
    def __init__(self, position):
        self.g = 0                    # g(n) where n is the RHS.
        self.h = 0                    # h(n) where n is the RHS.
        self.cost = self.g + self.h   # f(n) = g(n) + h(n)
        self.position = position      # robot's position [x, y]
        self.parent = None            # Parent Node pointer.

    def update_cost(self):
        self.cost = self.h + self.g

    def get_g(self):
        return self.g

    def set_g(self, g):
        self.g = g

    def get_h(self):
        return self.h

    def set_h(self, h):
        self.h = h

    def get_cost(self):
        return self.cost

    def set_cost(self, cost):
        self.cost = cost

    def get_position(self):
        return self.position

    def set_position(self, position):
        self.position = position

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent