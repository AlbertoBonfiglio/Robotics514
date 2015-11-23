#!/usr/bin/env python


class KalmanFilter:
    def __init__(self, a, b, c, r, q, mean=0.0, variance=1000.0):
        self.a = a
        self.b = b
        self.c = c
        self.r = r
        self.q = q
        self.mean = mean
        self.variance = variance

    def update(self, u, z):
        mean_prediction = self.a * self.mean + self.b * u
        variance_prediction = self.a * self.a * self.variance + self.r
        kalman_gain = self.variance * self.c /  \
                      (self.c * self.c * self.variance + self.q)
        self.mean = mean_prediction +  \
                    kalman_gain * (z - self.c * mean_prediction)
        self.variance = (1.0 - kalman_gain * self.c) * variance_prediction


if __name__ == '__main__':
    kf = KalmanFilter(1.0, 1.0, 1.0, 0.0, 1.0)

    x = 1.23
    
    for i in xrange(1000):
        u = 1.0
        x += u
        z = x
        
        kf.update(u, z)
        print 'Error:', x - kf.mean


