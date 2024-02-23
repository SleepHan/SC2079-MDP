from tsp import TSP
import numpy as np

# Create TSP object - TSP(StartPos, XLen, YLen, TurnRad)
tsp = TSP((2, 2, 'North'), 20, 20, 2.25)
tsp.testObstacles(6)

# Calculate path dist - calcDubins(Step)
res = tsp.calcDubins(1)
tsp.printInfo(2)

if res:
    # Get sequence of obstacles to visit
    sequence, dist = tsp.calcTSP()
    print(dist)

    if dist != float('inf'):
        # Get the command to send to STM
        segments = tsp.generateCommands(sequence, len(sequence))

        # Seg = [Command, Rad, Dist, StartCoor, EndCoor]
        for seg in segments:
            print('{}: {}'.format(seg[0], round(seg[2 if seg[0] == 'S' else 1], 2)))

            print('{} -> {}'.format(seg[3], seg[4]))
            print()

        # Simulator
        # tsp.displayPath(sequence, len(sequence))
    
    else:
        print('No valid path')