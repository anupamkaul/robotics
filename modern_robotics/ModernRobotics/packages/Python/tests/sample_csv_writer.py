# since many CoppeliaSim inputs are CSV files
# here's a quick way to create CSV files in python

import numpy as np

# generate random 3*4 matrix of floats y, 3*1 vector of ints d
y = np.random.rand(3, 4) # defaults to float
d = np.random.randint(-100, 100, 3)

#print(y)
#print(d)

# open a file for output csv
# mode: overwrite
f = open("output.csv", "w")
# mode: append
#f = open("output.csv", "w")

# For loop running 3 times to print each csv row
for i in range(len(d)):
    output = " %10.6f, %10.6f, %10.6f, %10.6f, %d\n" % (y[i,0], y[i,1], y[i,2], y[i,3], d[i])
    f.write(output)

# close file
f.close()

