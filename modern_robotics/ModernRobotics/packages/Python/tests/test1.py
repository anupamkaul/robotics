## import the package
import modern_robotics as mr

## use some functions - use the helper function for RotInv from package:
help(mr.RotInv)

##next try this code: (q to quit)
import numpy as np

R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, 1, 0]])
invR = mr.RotInv(R)
print ("Var invR = \n",  invR)






