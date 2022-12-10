import numpy as np

class SlewFilter:
    
    def __init__(self, sample_hz, max_change_per_sec, value):
        # compute max change per iteration based on inputs
        self.max_change_per_iteration = max_change_per_sec/sample_hz
        # store initial value to be refereced as previous value each timestep
        self.prev_value = value

    def filter(self, value):
        # if abs(value - self.prev_value) > self.max_change_per_iteration:
        #     control = value + np.sign(value - self.prev_value) * self.max_change_per_iteration
        #     self.prev_value = value
        #     return control
        # self.prev_value = value
        return value
