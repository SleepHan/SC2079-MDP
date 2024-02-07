"""
fastest car task using visual recognition 

Map : 200 by 200

Robot has a footprint of 20cm x 21cm

The best position is to have the camera 20cm away from the obstacle to recognize an image.

obstacle will be placed at the centre of the carpark
- obstacle 1 : 10 x 10
- obstacle 2 : 60 x 10

parking barrier will be 60cm wide

wall/barrier is approximately 50cm away from obstacle 2

right / left arrow only

Time out for this task is 3min.
"""

# imports
# import robot configuration file
# import directions file
# ....

# class with an empty constructor
class FastestPath:
    def __init__(self):
        pass

    def start(self, robot, way_point, goal, speed, on_grid, move):
        a_star_path_finder = AStarPathFinder() 
        a_star_path_finder.set_direction(robot.get_direction())
        a_star_path_finder.set_first(True)
        path, path1, path2 = None, None, None

        if a_star_path_finder.is_valid(robot, way_point):
            if not move:
                a_star_path_finder.set_first_turn_penalty(False)
            path = a_star_path_finder.start(robot, robot.get_position(), way_point, on_grid)
            if path:
                # checks whether the path is valid, sets a penalty
                a_star_path_finder.set_first_turn_penalty(True)
                # constructs the path using the start method of the AStarPathFinder class
                path1 = path
                path2 = a_star_path_finder.start(robot, way_point, goal, on_grid)
                path = path1 + path2
        else:
            # if path is not vaid
            a_star_path_finder.set_first_turn_penalty(True)
            path = a_star_path_finder.start(robot, robot.get_position(), goal, on_grid)

        if path and move:
            # checks if there is a path and if the robot should move
            if CommunicationSocket.check_connection() and FastestPathThread.is_running():
                self.fastest_path_movement_constructor(path, robot)
            else:
                self.move(robot, path, speed)

        print(path)
        print(robot.get_way_point())
        print("Finished Fastest Path")
        return path
    
    def fastest_path_movement_constructor(self, path, robot):
        sb = []
        instruction_count = 0
        for direction in path:
            if direction == Directions.FORWARD:
                instruction_count += 1
            elif instruction_count > 0:
                sb.append(f"F{instruction_count}|")
                if direction == Directions.RIGHT:
                    sb.append(RobotConfig.TURN_RIGHT)
                    instruction_count = 1
                elif direction == Directions.LEFT:
                    sb.append(RobotConfig.TURN_LEFT)
                    instruction_count = 1
                elif direction == Directions.BACKWARD:
                    sb.append(RobotConfig.TURN_RIGHT * 2)
                    instruction_count = 1
                else:
                    print("Error!")
                    return
        if instruction_count > 0:
            sb.append(f"F{instruction_count}|")
        msg = "".join(sb)
        robot.display_message("Message sent for start real run: " + msg)
        try:
            time.sleep(1)
            CommunicationSocket.get_instance().send_message(msg)
        except InterruptedException as e:
            print(e)

    def move(self, robot, path, speed):
        exploration = Exploration()
        for direction in path:
            if not CommunicationSocket.check_connection():
                try:
                    time.sleep(speed)
                except Exception as e:
                    print(e)

            if direction == Directions.FORWARD:
                if exploration.check_front_empty(robot):
                    robot.forward(1)
                else:
                    return
            elif direction == Directions.RIGHT:
                robot.update_map()
                robot.rotate_right()
                if exploration.check_front_empty(robot):
                    robot.forward(1)
                else:
                    return
            elif direction == Directions.LEFT:
                robot.update_map()
                robot.rotate_left()
                if exploration.check_front_empty(robot):
                    robot.forward(1)
                else:
                    return
            elif direction == Directions.BACKWARD:
                robot.update_map()
                robot.rotate_right()
                robot.update_map()
                robot.rotate_right()
                if exploration.check_front_empty(robot):
                    robot.forward(1)
                else:
                    return
            else:
                return

        robot.update_map()