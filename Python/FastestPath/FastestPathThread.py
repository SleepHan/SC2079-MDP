# imports
from AtomicBoolean import AtomicBoolean
from FastestPath import FastestPath
class FastestPathThread(Thread):
    """defines class-level variables isCompleted, isRunning, 
    and thread using AtomicBoolean for thread-safe boolean operations"""
    isCompleted = AtomicBoolean(False)
    isRunning = AtomicBoolean(False)
    thread = None

    def __init__(self, robot, way_point, speed):
        """sets the class-level isRunning to True, 
        checks if the robot is a SimulatedRobot, and starts the thread"""
        super().__init__(name="FastestPathThread")
        self.speed = speed
        self.way_point = way_point
        self.robot = robot
        FastestPathThread.isRunning.set(True)
        self.is_simulated = isinstance(robot, SimulatedRobot)
        self.start()

    @classmethod
    # returns an instance of the FastestPathThread. 
    # It ensures that only one instance of the thread is created
    def get_instance(cls, robot, way_point, speed):
        if cls.thread is None:
            cls.thread = FastestPathThread(robot, way_point, speed)
        return cls.thread

    @classmethod
    def is_completed(cls):
        return cls.isCompleted.get()

    @classmethod
    def is_running(cls):
        return cls.isRunning.get()

    def run(self):
        """ run method, which is executed when the thread is started 
         It creates an instance of FastestPath and runs the path planning. 
         It then updates the isCompleted and calls stop_thread. 
         After path planning, it sends messages or updates the simulated robot accordingly."""
        fastest_path = FastestPath()
        fastest_path.start(self.robot, self.way_point, RobotConfig.END, self.speed, True, True)
        FastestPathThread.isCompleted.set(FastestPathThread.isRunning.get())
        self.stop_thread()

        if CommunicationSocket.check_connection():
            CommunicationSocket.get_instance().send_message(RobotConfig.END_TOUR)
            self.robot.display_message("Sent message: " + RobotConfig.END_TOUR)
        else:
            if self.is_simulated:
                simulated_robot = self.robot
                simulated_robot.display_message("Fastest Path Completed")
                simulated_robot.set_direction(Directions.NORTH)

    @classmethod
    def stop_thread(cls):
        cls.thread = None
        cls.isRunning.set(False)