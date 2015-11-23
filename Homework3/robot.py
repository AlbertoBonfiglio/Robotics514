#!/usr/bin/env python


from numpy.random import normal


class Robot:
    def __init__(self, sensor_variance=0.1, movement_variance=0.1, x=0.0):
        self.x = x
        self.sensor_variance = sensor_variance
        self.movement_variance = movement_variance

    def move(self, action):
        self.x += action + normal(0.0, self.movement_variance)

    def sensor(self):
        return self.x + normal(0.0, self.sensor_variance)


if __name__ == '__main__':
    robot = Robot()

    for i in xrange(10):
        print robot.x, robot.sensor()
        robot.move(1.0)
