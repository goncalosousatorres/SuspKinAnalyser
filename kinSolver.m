function err = kinSolver(x, P, ConsMat,LockMat,distVec)

%1)update hardpoint matrix
P(~LockMat) = x(:);

%2) calculate new distances from constraint equations
dMat = squareform(pdist(P'));
dVec = dMat(ConsMat);

%3) calculate errors
err = abs(dVec - distVec);

end
