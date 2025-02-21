import numpy as np
H = np.zeros((2, 3))
H[0:2, 0:2] = np.eye(2)
H[1][1] = 2
print(H)
print(np.zeros((2,1)))

R = np.diag([
    0.05, 0.0,
    0.0, 0.05
])
print(R)