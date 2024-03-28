from tsp import TSP

# Create TSP object - TSP(StartPos, XLen, YLen, Step, TurnRad, V-Offset, H-Offset, V-Ajust, H-Adjust, CalType)
print('Start')
tsp = TSP(
        initPosition=(2, 2, 'N'), 
        dimX=200, 
        dimY=200, 
        step=10, 
        turnRad=3, 
        offFV=2, 
        offFH=3, 
        offRV=3, 
        offRH=2,
        distCalType=2
    )
tsp.testObstacles(8)

res = tsp.run(False)

if res:
    paths = tsp.getCommands()
    # Forward:  Straight, Left, Right - W, A, D
    # Backward: Straight, Left, Right - T, F, H
    for path in paths:
        print(path)
        print()