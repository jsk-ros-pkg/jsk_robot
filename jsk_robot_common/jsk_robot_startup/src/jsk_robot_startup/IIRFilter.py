from scipy import signal
from collections import deque

# Infinite Impulse Filter
# y[n] = sum(0, dim, ff[i] * x[n - i]) + sum(1, dim, fb[i] * y[n - i])
# dimensioon: dimension of filter
# cutoff_per_sampling: cutoff freq [Hz] / sampling freq [Hz]
class IIRFilter:
    def __init__(self, dimension, cutoff_per_sampling):
        self.dimension = dimension
        self.ff, self.fb = signal.butter(dimension, cutoff_per_sampling, "low")
        self.prev_values = deque([0.0] * dimension)

    def execute(self, input_value):
        feedback = self.fb[0] * input_value
        for i in range(self.dimension):
            feedback -= self.fb[i + 1] * self.prev_values[i]
        filtered = self.ff[0] * feedback
        for i in range(self.dimension):
            filtered += self.ff[i + 1] * self.prev_values[i]
            
        self.prev_values.appendleft(feedback) 
        self.prev_values.pop() # remove oldest value
        
        return filtered
    
    def reset(self):
        self.prev_values = deque([0.0] * self.dimension)
    
