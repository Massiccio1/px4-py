# https://old.reddit.com/r/learnpython/comments/daow1f/how_to_plot_realtime_data_on_matplotlib/

import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
# https://docs.python.org/3/library/collections.html#collections.deque
from collections import deque

TIMESTEP = 100 # 100ms = 0.1s (but not guaranteed see tkinter.after())
MAX_IDX = 100 # number of data points in sliding plot 100 * 0.1s = 10s of data

class GUI:
    def __init__(self, master):
        self.master = master
        self.X = deque()
        self.Y = deque()
        self.fig = Figure(figsize = (4, 3))
        self.axis = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master = self.master)
        self.canvas._tkcanvas.pack(side = tk.TOP, fill = tk.BOTH, expand = 1)
        self.loop()

    def loop(self):
        self.master.after(TIMESTEP, self.loop)
        if len(self.X) >= MAX_IDX:
            self.X.popleft()
            self.Y.popleft()
        try:
            self.X.append(self.X[-1]+TIMESTEP) # -1: last element
            self.axis.set_xlim(min(self.X), max(self.X))
        except IndexError:
            self.X.append(0) # first element
        self.Y.append(random.uniform(0.0, 2.7))
        self.axis.set_ylim(min(self.Y)-0.25, max(self.Y)+0.25)
        self.axis.grid(which='both')
        self.axis.plot(self.X, self.Y, color='blue')
        self.canvas.draw()
        #print(f"{len(self.X)}\t{self.X[-1]:.1f}, {self.Y[-1]:.1f}")

root = tk.Tk()
root.title("Moving Plot Demo")
gui = GUI(root)
root.mainloop()