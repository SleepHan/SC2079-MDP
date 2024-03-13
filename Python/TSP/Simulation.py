from tsp import TSP
import numpy as np

# Create TSP object - TSP(StartPos, XLen, YLen, TurnRad)
print('Start')
tsp = TSP((1, 1, 'N'), 20, 20, 2.5)
tsp.testObstacles(4)
# print(tsp.obstacleList)
# print(tsp.positionsDir)

res = tsp.calAStar()

if res:
    seq, dist = tsp.calcTSP()
    print(dist)
    print(seq)
else:
    print('No valid path')

# Calculate path dist - calcDubins(Step)
# res = tsp.calcDubins(1)
# tsp.printInfo(2)

# if res:
#     # Get sequence of obstacles to visit
#     sequence, dist = tsp.calcTSP()
#     print(dist)

#     if dist != float('inf'):
#         # Get the command to send to STM
#         fullPath = tsp.generateCommands(sequence, len(sequence))

#         # Getting path to next obstacle in sequence
#         for path in fullPath:
#             # path = [SegmentList, StartCoorOfPath, EndCoorOfPath, ObstacleCoor]
#             segments, startCoor, endCoor, obsCoor = path
#             print('Path to {}: {} -> {}'.format(obsCoor, startCoor, endCoor))

#             # Seg = [Command, Rad, Dist, StartCoorOfSegment, EndCoorOfSegment]
#             for seg in segments:
#                 print('{}: {}'.format(seg[0], round(seg[2 if seg[0] == 'S' else 1], 2)))
#                 print('{} -> {}'.format(seg[3], seg[4]))
#                 print()

#             print()

#         # Simulator
#         # tsp.displayPath(sequence, len(sequence))
    
#     else:
#         print('No valid path')