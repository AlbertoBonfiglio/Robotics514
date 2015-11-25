#!/usr/bin/env python

import Tkinter as Tk
from kalman import KalmanFilter
from robot import Robot
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import pyplot as plt

import numpy as np

from Tkinter import *


class App(Frame):
    __kalman = None
    __robot = None

    def __init__(self, master):

        Frame.__init__(self, master)
        self.pack()

        self.f = Figure(figsize=(7, 4), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.f, master=self)
        self.canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)
        self.canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)
        self.a = self.f.add_subplot(111)

        self.a.axis([0, 999, 0, 1500])

        self.spinner_a = Spinbox(self, from_=0.01, to=1, increment=0.01)
        self.spinner_a.pack()
        self.spinner_b = Spinbox(self, from_=0.01, to=1,increment=0.01)
        self.spinner_b.pack()
        self.spinner_c = Spinbox(self, from_=0.01, to=1,increment=0.01)
        self.spinner_c.pack()

        self.spinner_r = Spinbox(self, from_=0.01, to=1, increment=0.01 )
        self.spinner_r.pack()
        self.spinner_q = Spinbox(self, from_=0.01, to=1, increment=0.01)
        self.spinner_q.pack()

        self.spinner_m = Spinbox(self, from_=0.01, to=1, increment=0.01)
        self.spinner_m.pack()
        self.spinner_v = Spinbox(self, from_=0.01, to=1000, increment=0.5)
        self.spinner_v.pack()



        self.button = Button(self, text="QUIT", fg="red", command=self.quit)
        self.button.pack(side=LEFT)

        self.button = Button(self, text="Save Graph", fg="red", command=self.takePicture)
        self.button.pack(side=LEFT)


        self.hi_there = Button(self, text="Set Values", command=self.updateKalman)
        self.hi_there.pack(side=LEFT)

        self.hi_there = Button(self, text="Run", command=self.runSimulation)
        self.hi_there.pack(side=LEFT)



    def updateKalman(self):
        try:
            self.__kalman = KalmanFilter(float(self.spinner_a.get()), float(self.spinner_b.get()), float(self.spinner_c.get()), \
                                         float(self.spinner_q.get()), float(self.spinner_r.get()),
                                         float(self.spinner_m.get()), float(self.spinner_v.get())
                                         )


            print(self.spinner_a.get())
        except Exception as ex:
            print(ex.message)

    def runSimulation(self):
        self.__robot = Robot()
        self.updateKalman()
        rng = 1000
        rX = np.array([]);
        fX = np.array([]);

        for i in xrange(rng):
            self.__robot.move(1.5)
            self.__kalman.update(1, self.__robot.sensor())

            rX = np.append(rX, self.__robot.x)
            fX = np.append(fX, self.__kalman.mean)

            # Print the robot's actual position vs the filter estimate
            print self.__robot.x, self.__kalman.mean

        xscale = range(0, rng)

        self.a.clear()
        self.a.set_ylabel('Robot distance')
        self.a.set_xlabel('Iterations')
        self.a.set_title('Kalman Filter')

        #self.a.plot(xscale, rX, "green",  xscale, fX, "r.-", label='icoli', label='Test')
        self.a.plot(xscale, rX, "g--",  linewidth=2, label='Path')
        self.a.plot(xscale, fX, "r-.", linewidth=2, label='Filter')
        handles, labels = self.a.get_legend_handles_labels()
        self.a.legend(handles, loc = "upper_left", frameon = True, labelspacing = 0)

        # Specific axis choice (get rid of default compression on Y!)



        self.canvas.show()

    def takePicture(self):
        self.f.savefig('hello.png')


#if __name__ == '__main__':
root = Tk()

app = App(root)

root.mainloop()
#root.destroy() # optional; see description below