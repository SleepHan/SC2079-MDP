from tsp import TSP

# Create TSP object - TSP(StartPos, XLen, YLen, Step, TurnRad, V-Offset, H-Offset, V-Ajust, H-Adjust, CalType)
print('Start')
tsp = TSP((1, 1, 'N'), 200, 200, 10, 3, 3, 3, 5, 0, 2)
tsp.testObstacles(2)

res = tsp.run()

if res:
    paths = tsp.getCommands()
    # Forward:  Straight, Left, Right - W, A, D
    # Backward: Straight, Left, Right - T, F, H
    for path in paths:
        print(path)

    # for path in paths:
    #     # path = ([Segments], startPath, endPath, targetObstacle)
    #     segment_list = path[0]
    #     obstacle_coor = path[-1]
    #     segment_list.append({
    #         'type': 'SEG_END',
    #         'obs':  obstacle_coor
    #     })
    # algo_commands = [segment for path in tsp.commands for segment in path[0]]

    # if len(algo_commands) > 0:
    #     algo_command = algo_commands.pop(0)
    #     print("Current algo command: {}".format(algo_command))
    # else:
    #     print("Algorithm done executing")
    #     algo_in_control = False
    
    # # Segment = { type, startSeg, endSeg, length }          - Straight Movements
    # # Segment = { type, startSeg, endSeg, angle, length}    - Turn Movements
    # end_of_segment_reached = False
    # if algo_commands[0]['type'] == 'SEG_END':
    #     obstacle_coor = algo_commands[0]['obs']
    #     algo_commands.pop(0)
    #     end_of_segment_reached = True

    # if algo_command['type'] == 'AW':
    #     to_stm = 'd{}{}'.format(min(round(algo_command['length']), 99), min(round(algo_command['angle']), 99))
    # elif algo_command['type'] == 'DW':
    #     to_stm = 'a{}{}'.format(min(round(algo_command['length']), 99), min(round(algo_command['angle']), 99))
    # elif algo_command['type'] == 'W':
    #     to_stm = 'w{}00'.format(min(round(algo_command['length']), 99))
    # elif algo_command['type'] == 'S':
    #     to_stm = 't{}00'.format(min(round(algo_command['length']), 99))

    #     print('=========New Path=========')
    #     for seg in path[0]:
            
    #         print(seg)
    #     print('==========================')