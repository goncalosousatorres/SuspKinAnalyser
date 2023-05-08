import scipy as sp
import numpy as np

def kinCost(x, *data):
    P, ConsMat, LockMat, distVec = data
    P[~LockMat] = x
    dMat = sp.spatial.distance.squareform(sp.spatial.distance.pdist(np.transpose(P)))
    dVec = dMat[ConsMat]
    return (dVec - distVec)**2
