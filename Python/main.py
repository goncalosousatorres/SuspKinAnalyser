import numpy as np
import scipy as sp
from kinCost import kinCost
import matplotlib.pyplot as plt

'''Hardpoint definition'''

# upper wishbone
P1 = np.array([-90.99, 248.40, 229.52])  # upper wishbone front pivot
P2 = np.array([93.51, 248.40, 232.76])  # upper wishbone rear pivot
P3 = np.array([10.33, 580.92, 289.53])  # upper wishbone outer ball joint

# lower wishbone
P4 = np.array([-96.60, 219.66, 93.44])  # lower wishbone front pivot
P5 = np.array([76.97, 219.66, 105.60])  # lower wishbone rear pivot
P6 = np.array([-5.94, 593.18, 103.53])  # lower wishbone outer ball joint

# damper axis
P7 = np.array([10.33, 162.54, 570.00])  # damper body end
P8 = np.array([10.33, 527.89, 299.63])  # damper wishbone end

# tie-rod
P9  = np.array([32.00, 217.97, 119.76])  # inner track rod ball joint
P10 = np.array([62.00, 550.00, 133.55])  # outer track rod ball joint

# spring axis
P11 = np.array([10.33, 162.54, 570.00])  # body spring pivot point
P12 = np.array([10.33, 527.89, 299.63])  # wishbone spring pivot point

# wheel axis
P13 = np.array([0.00, 585.00, 196.53])  # wheel spindle point
P14 = np.array([0.00, 615.00, 196.53])  # wheel centre point

# #tyre
# P15 = np.array([0.00, 615.00; 0.00])  # contact patch

HP = np.array([P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14])
HP = np.transpose(HP)

'''Unit vectors'''
i_hat = np.array([1, 0, 0])
j_hat = np.array([0, 1, 0])
k_hat = np.array([0, 0, 1])

'''Static Suspension Metrics'''
aKingpin0 = 90-np.rad2deg((np.arccos(np.dot(HP[:, 5]-HP[:, 2], j_hat)/np.linalg.norm(HP[:, 5]-HP[:, 2]))))
aCaster0 = 90-np.rad2deg((np.arccos(np.dot(HP[:, 5]-HP[:, 2], -i_hat)/np.linalg.norm(HP[:, 5]-HP[:, 2]))))
aCamber0 = -90+np.rad2deg(np.arccos(np.dot(HP[:, 13]-HP[:, 12], k_hat)/np.linalg.norm(HP[:, 13]-HP[:, 12])))
aToe0 = 90-np.rad2deg(np.arccos(np.dot(HP[:, 13]-HP[:, 12], i_hat)/np.linalg.norm(HP[:, 13]-HP[:, 12])))
lKingpin = np.linalg.norm(HP[:, 5]-HP[:, 2])
lTieRod = np.linalg.norm(HP[:, 9]-HP[:, 8])
# todo: scrub radius, mechanical trail, kingpin offset

'''Constraints and Locked Points'''
# Create constraint matrix
ConsMat = np.zeros((HP.shape[1], HP.shape[1]), dtype='bool')
ConsMat[2, 0:2] = True  # upper ball joint constraint
ConsMat[5, 2:4] = True  # lower ball joint constraint
ConsMat[7, 0:3] = True  # damper wishbone constraint
ConsMat[9, [2, 5, 8]] = True  # tie-rod constraint
ConsMat[11, 0:3] = True  # spring wishbone constraint
ConsMat[12, [2, 5, 9]] = True  # spindle constraint
ConsMat[13, [2, 5, 9, 12]] = True  # wheel center constraint

# Distance Matrix
DistMat = sp.spatial.distance.squareform(sp.spatial.distance.pdist(np.transpose(HP)))
DistVec = DistMat[ConsMat]

# Locked points matrix (tells which coordinates are "free" to move)
LockMat = np.zeros(HP.shape, dtype='bool')
LockMat[:, [0, 1, 3, 4, 6, 8, 10]] = True

'''Vertical Displacement Solution'''
# Sweep of wheel centre vertical displacement
dZ = np.arange(-25, 26, 1)

# Lock wheel centre z coordinate (as this is defined by the sweep array)
LockMat[2, -1] = True

# Create empty vectors to store useful quantities
aCamber = np.empty(dZ.shape)
aToe = np.empty(dZ.shape)
aKingpin = np.empty(dZ.shape)
aCaster = np.empty(dZ.shape)

for i, z in enumerate(dZ):
    # 1) update wheel centre coordinate
    HP[2, -1] = P14[2] + z

    # 2) create initial guess from static solution
    initGuess = HP[~LockMat]

    # 3) solve system of nonlinear equations
    data = (HP, ConsMat, LockMat, DistVec)
    xSol = sp.optimize.fsolve(kinCost, initGuess, args=data)

    # 4) extract solution
    HP[~LockMat] = xSol

    # 5) Calculate metrics and store in array
    aKingpin[i] = 90 - np.rad2deg((np.arccos(np.dot(HP[:, 5] - HP[:, 2], j_hat) / np.linalg.norm(HP[:, 5] - HP[:, 2]))))
    aCaster[i] = 90 - np.rad2deg((np.arccos(np.dot(HP[:, 5] - HP[:, 2], -i_hat) / np.linalg.norm(HP[:, 5] - HP[:, 2]))))
    aCamber[i] = -90 + np.rad2deg(np.arccos(np.dot(HP[:, 13] - HP[:, 12], k_hat) / np.linalg.norm(HP[:, 13] - HP[:, 12])))
    aToe[i] = 90 - np.rad2deg(np.arccos(np.dot(HP[:, 13] - HP[:, 12], i_hat) / np.linalg.norm(HP[:, 13] - HP[:, 12])))

plt.plot(dZ, aToe)
plt.show()





