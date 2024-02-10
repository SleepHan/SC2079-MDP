from fastest_path_algo import FastestPathAlgo
from exploration_algo import ExplorationAlgo


class Core:
    def __init__(self, handler):
        self.handler = handler
        self.map = self.handler.map
        self.path_finder = FastestPathAlgo(self.map, self.handler.robot, self.handler)
        self.explorer = ExplorationAlgo(self.handler, self.path_finder)

    def reset(self):
        self.explorer.reset()

    def explore(self, steps_per_second, coverage, time_limit, exploration_algo, perform_fp=False):
        is_return_home = 'Return Home' in exploration_algo

        if steps_per_second == -1:
            delay = 10
        else:
            delay = 1000 // steps_per_second

        if 'Image Recognition' in exploration_algo:
            if 'Partial' in exploration_algo:
                self.explorer.set_status(do_img_rec=True, partial_ir=True)
            else:
                self.explorer.set_status(do_img_rec=True, partial_ir=False)
        else:
            if 'Optimized' in exploration_algo:
                self.explorer.set_optimized(True)
            else:
                self.explorer.set_optimized(False)
            self.explorer.set_status(do_img_rec=False, partial_ir=False)
        self.explorer.sense()
        self.explorer.explore(delay, steps_per_second, coverage, time_limit, is_return_home, perform_fp=perform_fp)

    def findFP(self, steps_per_second, goal_x, goal_y, waypoint_x, waypoint_y, fp_algo):
        if steps_per_second == -1:
            delay = 10
        else:
            delay = 1000 // steps_per_second

        if fp_algo == "A* Search":
            self.path_finder.find_fastest_path(diag=False, delay=delay, goalX=goal_x, goalY=goal_y,
                                               waypointX=waypoint_x,
                                               waypointY=waypoint_y)
        elif fp_algo == "A* Search (With Diagonals)":
            self.path_finder.find_fastest_path(diag=True, delay=delay, goalX=goal_x, goalY=goal_y,
                                               waypointX=waypoint_x,
                                               waypointY=waypoint_y)
        else:
            self.explore(steps_per_second, 100, 3600, "Left Wall Hugging", perform_fp=True)