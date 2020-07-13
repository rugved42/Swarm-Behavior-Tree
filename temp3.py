from geneticalgorithm1 import geneticalgorithm as ga
import numpy as np

def f(X):
    return np.sum(X)


varbound=np.array([[0,10]]*3)

model=ga(function=f,dimension=3,variable_type='real',variable_boundaries=varbound)

model.run()