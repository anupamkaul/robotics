## import the package
import modern_robotics as mr

## use some functions - use the helper function for RotInv from package: (q to quit help)
help(mr.RotInv)
help(mr.VecToso3)

##next try this code: 
import numpy as np

# -- show inverse of a rotation matrix --
# R : Rotation Matrix (defined with numpy)
# invR : Inverse of R
# (for efficiency, inverse is calculated as a transpose rather than a matrix inverse)

R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, 1, 0]])
invR = mr.RotInv(R)
print ("Var invR = \n",  invR, "\n")

# -- show VecToso3(omg) --
# Input: omg : A 3-vector
# Output : so3mat: The corresponding 3*3 skew-symmetric matrix in so(3)

omg = np.array([1,2,3])
so3 = mr.VecToso3(omg)

print("Var so3 = \n", so3, "\n")














