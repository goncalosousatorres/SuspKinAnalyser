function plotTyre(WC,rTyre, aCamber, aToe)

%%
aCamber = -aCamber;
aToe = -aToe;

rxMat = [1,             0,              0;
         0, cosd(aCamber), -sind(aCamber);
         0, sind(aCamber), cosd(aCamber)];

rzMat = [cosd(aToe), -sind(aToe), 0;
         sind(aToe),  cosd(aToe), 0;
                  0,           0, 1];

rMat = rzMat*rxMat;

j = linspace(0,2*pi,50);

PTyre = [rTyre*cos(j); 
         zeros(1,50);
         rTyre*sin(j)];

PTyre = rMat*PTyre + WC;

plot3(PTyre(1,:),PTyre(2,:), PTyre(3,:),'--k');

% TopTyre = CP + 2*rTyre*[sind(aCamber)*cosd(aToe);
%                  sind(aCamber)*sind(aToe);
%                  cos(aCamber)];
% 
% plot3([TopTyre(1), CP(1)],[TopTyre(2),CP(2)],[TopTyre(3),CP(3)],'-.k')