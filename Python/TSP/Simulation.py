from tsp import TSP
import numpy as np

tsp = TSP((2, 2, 'North'), 20, 20, 2.25)
tsp.testObstacles(5)

res = tsp.calcDubins(.5)
tsp.printInfo(2)

if res:
    sequence, dist = tsp.calcTSP()
    # Simulator
    # tsp.displayPath(sequence, len(sequence))
    
    # Command - Dist/Rad
    segments, path = tsp.printPathInfo(sequence, len(sequence))
    # for seg in segments:
    #     print('{}:\t{}'.format(seg[0], round(seg[2], 2)))

    # Points
    print(path)