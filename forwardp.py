import numpy as np

class Node:
    def __init__(self):
        self.w1 = 0.5
        self.w2 = 0.5
        self.bias = 0.6
        self.weight_matrix = np.transpose(np.matrix([self.w1, self.w2]))
                                          

    def compute(self, x):
        ## forward propage out = W^t * x
        out = np.dot(self.weight_matrix, np.array(x).T)
        if out > self.bias:
            return 1
        else:
            return 0
      


n = Node()
print(n.compute([1,0]))

    
