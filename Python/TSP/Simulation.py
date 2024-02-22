from tsp import TSP
import numpy as np

tsp = TSP((2, 2, 'North'), 20, 20, 2.5)
tsp.testObstacles(2)

res = tsp.calcDubins(.5)
tsp.printInfo(2)

if res:
    sequence, dist = tsp.calcTSP()
    tsp.displayPath(sequence, len(sequence))
    # segments = tsp.printPathInfo(sequence, len(sequence))
    # for seg in segments:
    #     print('{}:\t{}'.format(seg[0], round(seg[2], 2)))
