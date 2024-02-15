from tsp import TSP

tsp = TSP((2, 2, 'North'))
tsp.testObstacles()

tsp.calcDubins(2.5, .5)
tsp.printInfo(2)
tsp.printInfo(3)

sequence, dist = tsp.calcTSP()
tsp.displayPath(sequence, len(sequence))
# tsp.printPathInfo(sequence, len(sequence))