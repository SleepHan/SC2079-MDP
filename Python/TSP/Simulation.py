from fastestcar import FastestCar

fastestcar = FastestCar((2, 10, 'East'))


fastestcar.testObstacles()
fastestcar.calcDubins(2.5, .5)
fastestcar.printInfo(2)
fastestcar.printInfo(3)
sequence, dist = fastestcar.calcTSP()
fastestcar.displayPath(sequence, len(sequence))
        

# from tsp import TSP
# from fastestcar import FastestCar

# class Simulation:
#     def __init__(self):
#         self.tsp = TSP((2, 2, 'North'))
#         self.fastestcar = FastestCar((2, 10, 'North'))
        

#     def run_exploration(self):
#         self.tsp.testObstacles()
#         self.tsp.calcDubins(2.5, .5)
#         self.tsp.printInfo(2)
#         self.tsp.printInfo(3)
#         sequence, dist = self.tsp.calcTSP()
#         self.tsp.displayPath(sequence, len(sequence))

#     def run_fastest_car(self):
#         self.fastestcar.testObstacles()
#         self.fastestcar.calcDubins(2.5, .5)
#         self.fastestcar.printInfo(2)
#         self.fastestcar.printInfo(3)
#         sequence, dist = self.tsp.calcTSP()
#         self.fastestcar.displayPath(sequence, len(sequence))
        

# if __name__ == "__main__":
#     simulation = Simulation()

#     # GUI code (Tkinter)
#     import tkinter as tk

#     def exploration_button_callback():
#         simulation.run_exploration()

#     def fastest_car_button_callback():
#         simulation.run_fastest_car()

#     root = tk.Tk()
#     exploration_button = tk.Button(root, text="Exploration", command=exploration_button_callback)
#     exploration_button.pack()

#     fastest_car_button = tk.Button(root, text="Fastest Car", command=fastest_car_button_callback)
#     fastest_car_button.pack()

#     root.mainloop()