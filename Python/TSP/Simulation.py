from tsp import TSP

tsp = TSP((2, 2, 'North'), 20, 20)
tsp.testObstacles()

res = tsp.calcDubins(2.5, .5)
tsp.printInfo(2)

if res:
    sequence, dist = tsp.calcTSP()
    tsp.displayPath(sequence, len(sequence))
    # tsp.printPathInfo(sequence, len(sequence))