import time
from ulab import numpy as np

class KF:
    def __init__(self):
        
        self.x = np.array([0.,0.,0.,0.,0.,0.]).T
        self.t_prev = time.time()
    def predict(self,acc):
        DT = self.compute_DT()
        F = np.array([[1,0,0,DT,0,0],   #state transition matrix
                            [0,1,0,0,DT,0],
                            [0,0,1,0,0,DT],
                            [0,0,0,1,0,0],
                            [0,0,0,0,1,0],
                            [0,0,0,0,0,1]])
        B = np.array([[(DT**2)/2,0,0],    #control transition matrix
                             [0,(DT**2)/2,0],
                             [0,0,(DT**2)/2],
                             [DT,0,0],
                             [0,DT,0],
                             [0,0,DT]])
        
        self.x = np.dot(F,self.x) + np.dot(B, acc) # acc shape = 3X1
        
    def update(self,x):
        self.t_prev=time.time()
        self.x = x
    
    def compute_DT(self):
        t_curr = time.time()
        DT = t_curr-self.t_prev
        self.t_prev=t_curr
        return DT