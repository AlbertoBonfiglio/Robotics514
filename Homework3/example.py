#!/usr/bin/env python

from robot import Robot
from kalman import KalmanFilter
from matplotlib import pyplot as plt
import numpy as np


if __name__ == '__main__':
    robot = Robot()
    #filter = KalmanFilter(10.0, 10.0, 10.0, 10.0, 10.0, 10.0)
    filter = KalmanFilter(1.0, 1.0, 1.0, 1.0, 1.0, 120, 1000)
    rX = np.array([]);
    fX = np.array([]);

    for i in xrange(1000):
        robot.move(1.0)
        filter.update(1.0, robot.sensor())
        rX = np.append(rX, robot.x)
        fX = np.append(fX, filter.mean)

        # Print the robot's actual position vs the filter estimate
        print robot.x, filter.mean

    xscale = range(0,1000)
    print len(xscale)
    print len(rX)
    print len(fX)

    plt.plot(xscale, rX, "brown",
                xscale, fX, "r.-")

    # Specific axis choice (get rid of default compression on Y!)

    plt.axis([0, 999, 0, 1500])

    plt.ylabel('Accesses per hour')
    plt.xlabel('Hour of the day')
    plt.title('www.wellho.net daily accesses')
    plt.show()

