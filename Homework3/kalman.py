#!/usr/bin/env python
import numpy

class KalmanFilter:

    def __init__(self, a, b, c, q, r, mean=0.0, variance=1000.0):
        self.a = a
        self.b = b
        self.c = c

        self.q = q  # Process covariance
        self.r = r  # Sensor covariance

        self.mean = mean
        self.variance = variance

    def update(self, u, z):
        mean_prediction = self.a * self.mean + self.b * u
        variance_prediction = self.a * self.a * self.variance + self.q
        kalman_gain = self.variance * self.c /  \
                      (self.c * self.c * self.variance + self.r)
        self.mean = mean_prediction +  \
                    kalman_gain * (z - self.c * mean_prediction) #current_state_estimate
        self.variance = (1.0 - kalman_gain * self.c) * variance_prediction #current_prob_estimate



class KalmanFilterLinear:
    def __init__(self,_A, _B, _H, _x, _P, _Q, _R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.current_state_estimate = _x # Initial state estimate.
        self.current_prob_estimate = _P  # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements.

    def GetCurrentState(self):
        return self.current_state_estimate

    def Step(self, control_vector, measurement_vector):
        #---------------------------Prediction step-----------------------------
        predicted_state_estimate = self.A * self.current_state_estimate + self.B * control_vector
        predicted_prob_estimate = (self.A * self.current_prob_estimate) * numpy.transpose(self.A) + self.Q
        #--------------------------Observation step-----------------------------
        innovation = measurement_vector - self.H*predicted_state_estimate
        innovation_covariance = self.H*predicted_prob_estimate*numpy.transpose(self.H) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = predicted_prob_estimate * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
        self.current_state_estimate = predicted_state_estimate + kalman_gain * innovation
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = (numpy.eye(size)-kalman_gain*self.H)*predicted_prob_estimate
 # taken from http://greg.czerniak.info/guides/kalman1/

if __name__ == '__main__':
    kf = KalmanFilter(1.0, 1.0, 1.0, 0.0, 1.0)

    x = 1.23
    
    for i in xrange(1000):
        u = 1.0
        x += u
        z = x
        
        kf.update(u, z)
        print 'Error:', x - kf.mean


