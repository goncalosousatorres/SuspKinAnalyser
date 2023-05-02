%% Suspension Kinematics Solver
clc
clear
close all

%Constants and static parameters
rTyre = 196.53;           %tyre radius

%Hardpoint definition

%upper wishbone
P1 = [-90.99; 248.40; 229.52];         %upper wishbone front pivot
P2 = [93.51; 248.40; 232.76];          %lower wishbone front pivot
P3 = [10.33; 580.92; 289.53];          %upper wishbone outer ball joint


%lower wishbone
P4 = [-96.60; 219.66; 93.44];            %lower wishbone front pivot
P5 = [76.97; 219.66; 105.60];           %lower wishbone rear pivot
P6 = [-5.94; 593.18; 103.53];           %lower wishbone outer ball joint

%push-rod
P7 = [10.33; 162.54; 570.00];           %damper body end
P8 = [10.33; 527.89; 299.63];           %damper wishbone end

%track-rod
P9  = [32.00; 217.97; 119.76];          %inner track rod ball joint
P10 = [62.00; 550.00; 133.55];          %outer track rod ball joint

%spring
P11 = [10.33; 162.54; 570.00];          %body spring pivot point
P12 = [10.33; 527.89; 299.63];          %wishbone spring pivot point

%wheel
P13 = [0.00; 585.00; 196.53];           %wheel spindle point
P14 = [0.00; 615.00; 196.53];           %wheel centre point

%tyre
% P15 = [0.00; 615.00; 0.00];             %contact patch

HP = [P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14];

linefrmt = 'o-';
figure; subplot(2,4,[1,2,5,6]);
plotSusp(HP,linefrmt);



%% Static Suspension Metrics
aKingpin0 = 90-(acosd(dot(HP(:,6)-HP(:,3), [0;1;0])/norm(HP(:,6)-HP(:,3))));
aCaster0 = 90-(acosd(dot(HP(:,6)-HP(:,3), [-1;0;0])/norm(HP(:,6)-HP(:,3))));
aCamber0 = -90+(acosd(dot(HP(:,14)-HP(:,13), [0;0;1])/norm(HP(:,14)-HP(:,13))));
aToe0 = 90-(acosd(dot(HP(:,14)-HP(:,13), [1;0;0])/norm(HP(:,14)-HP(:,13))));
lKingpin = norm(HP(:,6)-HP(:,3));
lTieRod = norm(HP(:,10)-HP(:,9));
%todo: scrub radius, mechanical trail, kingpin offset

%% Constraints and Locked Points

%Create constraint matrix
ConsMat = logical(zeros(size(HP,2))); 
ConsMat(3,1:2) = 1;              %upper ball joint constraint
ConsMat(6,3:5) = 1;              %lower ball joint constraint
ConsMat(8,1:3) = 1;              %damper wishbone constraint
ConsMat(10,[3,6,9]) = 1;         %tie-rod constraint
ConsMat(12,1:3) = 1;             %spring wishbone constraint
ConsMat(13,[3,6,10]) = 1;        %spindle constraint
ConsMat(14,[3,6,10,13]) = 1;     %wheel center constraint

%Distance Matrix (created from constraint matrix)
DistMat = squareform(pdist(HP'));
DistVec = DistMat(ConsMat);

%Locked points matrix (tells which coordinates are "free" to move)
LockMat = logical(zeros(size(HP)));
LockMat(:,[1,2,4,5,7,9,11]) = 1;


%% Vertical displacement solution

aCamber = [];
aToe = [];
aKingpin = [];
aCaster = [];

options = optimoptions("fsolve",'Display','none','Algorithm','levenberg-marquardt');


for dZ = -25:1:25 %applied displacement to wheel center

%update wheel center coordinate and lock matrix
HP(3,end) = P14(3) + dZ;
LockMat(3,end) = 1;

%create initial guess from static solution (only "unlocked" coordinates)
x0 = HP(~LockMat);


% %Create arrays to store sweep
% dCamber = [];
% dTrack = [];


%Solve system of nonlinear equations
[sol,val] = fsolve(@(x) kinSolver(x, HP, ConsMat, LockMat,DistVec),x0,options);
x = sol;

%% Extract solution and plot
HP(~LockMat) = sol(:);

aKingpin =  [aKingpin, 90-(acosd(dot(HP(:,6)-HP(:,3), [0;1;0])/norm(HP(:,6)-HP(:,3))))];
aCaster = [aCaster, 90-(acosd(dot(HP(:,6)-HP(:,3), [-1;0;0])/norm(HP(:,6)-HP(:,3))))];
aCamber = [aCamber, -90+(acosd(dot(HP(:,14)-HP(:,13), [0;0;1])/norm(HP(:,14)-HP(:,13))))];
aToe = [aToe, 90-(acosd(dot(HP(:,14)-HP(:,13), [1;0;0])/norm(HP(:,14)-HP(:,13))))];

end

subplot(2,4,3); plot(-25:1:25,aKingpin); title('Kingpin Angle'); xlabel('Wheel Travel [mm]'); ylabel('Kingpin Angle [deg]');
subplot(2,4,4); plot(-25:1:25,aCaster); title('Caster Angle'); xlabel('Wheel Travel [mm]'); ylabel('Caster Angle [deg]');
subplot(2,4,7); plot(-25:1:25,aCamber); title('Camber Angle'); xlabel('Wheel Travel [mm]'); ylabel('Camber Angle [deg]');
subplot(2,4,8); plot(-25:1:25,aToe); title('Toe Angle'); xlabel('Wheel Travel [mm]'); ylabel('Toe Angle [deg]');






% lineformat = 'o--';
% plotSusp(HPsol,lineformat)





% 
% %Hardpoints
% P1n = P1;
% P2n = P2;
% P3n = [px3, py3];
% P4n = [px4, py4];
% P5n = [px5, P5(2) + dY];
% 
% dCamber = [dCamber, real(acosd(dot(P5-P4, P5n-P4n)/d4/d4))]; %camber gain
% dTrack = [dTrack, 2*(P5n(1) - P5(1))];
% 
% %update initial values
% initVals = [px3 py3 px4 py4 px5];
% 
% end
% toc
% 
% figure;
% 
% plot(-10:0.01:10, dCamber)
% 
% %% New Position
% 
% %Hardpoints
% P1n = P1;
% P2n = P2;
% P3n = [px3, py3];
% P4n = [px4, py4];
% P5n = [px5, P5(2) + dY];
% 
% %Link definition
% l1n = [P1n;P3n];           %upper link (wishbone)
% l2n = [P2n;P4n];           %lower link (wishbone)
% l3n = [P3n;P4n];           %oubj and olbj upright
% l4n = [P4n;P5n];           %olbj and contact patch
% l5n = [P3n;P5n];           %oubj and contact patch

% %Metrics
% dtrkWidth = P5n(1) - P5(1); %track width variation
% dCamber = real(acosd(dot(P5-P4, P5n-P4n)/d4/d4)); %camber gain


