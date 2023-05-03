function plotSusp(P,linefrmt,rTyre)

%upper wishbone
b1 = [P(:,1), P(:,2), P(:,3)];
plt.plotBody(b1, linefrmt, 'r'); hold on

%lower wishbone
b2 = [P(:,4), P(:,5), P(:,6)];
plt.plotBody(b2,linefrmt,'g');

%upright axis
b2 = [P(:,3), P(:,6), P(:,10)];
plt.plotBody(b2,linefrmt,'k');

%damper axis
b3 = [P(:,7), P(:,8)];
plt.plotBody(b3,linefrmt,'b');

%spring axis
b5 = [P(:,11), P(:,12)];
plt.plotBody(b5,linefrmt,'b');

%track-rod
b4 = [P(:,9), P(:,10)];
plt.plotBody(b4,linefrmt,'c');


%wheel axis
b6 = [P(:,13), P(:,14)];
plt.plotBody(b6,linefrmt,'k');

% aCamber0 = -90+(acosd(dot(HP(:,14)-HP(:,13), [0;0;1])/norm(HP(:,14)-HP(:,13))));
% aToe0 = 90-(acosd(dot(HP(:,14)-HP(:,13), [1;0;0])/norm(HP(:,14)-HP(:,13))));
% plt.plotTyre(P(:,14),rTyre,aCamber0,aToe0);

% bodies(:,:,1) = [P1;P2];            %chassis
% bodies(:,:,2) = [P1;P2];            %upper link (wishbone)
% bodies(:,:,3) = [P1;P2];            %lower link (wishbone)
% bodies(:,:,4) = [P1;P2];            %oubj and olbj upright
% bodies(:,:,5) = [P1;P2];            %chassis
% bodies(:,:,6) = [P1;P2];            %chassis
% 
% l0 = [P1;P2];           
% l1 = [P1;P3];           %upper link (wishbone)
% l2 = [P2;P4];           %lower link (wishbone)
% l3 = [P3;P4];           %oubj and olbj upright
% l4 = [P4;P5];           %olbj and contact patch
% l5 = [P3;P5];           %oubj and contact patch
% l6 = [P5;P6];           %wheel/tyre centerline
% 
% 
% 
% %Links' distances (euclidian)
% d1 = norm(P1-P3);
% d2 = norm(P2-P4);
% d3 = norm(P3-P4);
% d4 = norm(P4-P5);
% d5 = norm(P3-P5);
% 
% %Visualization
% f = figure(1);
% plot(l1(:,1), l1(:,2),'o-b','linewidth',3,'MarkerSize',6); hold on;
% plot(l2(:,1), l2(:,2),'o-g','linewidth',3,'MarkerSize',6);
% plot(l3(:,1), l3(:,2),'o-r','linewidth',3,'MarkerSize',6);
% plot(l4(:,1), l4(:,2),'o-r','linewidth',3,'MarkerSize',6);
% plot(l5(:,1), l5(:,2),'o-r','linewidth',3,'MarkerSize',6);
% plot(l0(:,1), l0(:,2),'o-k','linewidth',3,'MarkerSize',6);
% plot(l6(:,1), l6(:,2),'--k','linewidth',0.5);
% 
% text(P1(1)-1,P1(2)+1,'UIBJ');
% text(P2(1)-1,P2(2)-1,'LIBJ');
% text(P3(1)-1,P3(2)+1,'UOBJ');
% text(P4(1)-1,P4(2)-1,'LOBJ');
% text(P5(1)+1,P5(2)+1,'CP');
% 
% %Instant Center
% m1 = (P3(2)-P1(2))/(P3(1)-P1(1));
% m2 = (P4(2)-P2(2))/(P4(1)-P2(1));
% b1 = P3(2)-m1*P3(1);
% b2 = P4(2)-m2*P4(1);
% x = (b2-b1)/(m1+m2);
% y = m1*x+b1;
% 
% IC = [x, y];
% plot(IC(1), IC(2),'*K'); plot([IC(1), P1(1), P2(1), IC(1)],[IC(2), P1(2), P2(2), IC(2)],'--K');
% text(IC(1)-1,IC(2)+1,'IC');
% %Roll Center
% RCx = 0;                    %Assuming symmetric suspension geometry
% if ~isfinite(IC(1))
%     RCy = 0;
% else
%     mr = (P5(2)-y)/(P5(1)-x);
%     RCy = P5(2) - mr*P5(1);
% end
% 
% RC = [RCx, RCy];
% plot(RC(1),RC(2),'oK','MarkerSize',10);
% plot([IC(1),P5(1)], [IC(2),P5(2)],'--K');
% text(RC(1),RC(2)+1,'RC');
% 
% axis([IC(1)-10, P5(1)+10, P5(2)-10, P1(2) + 20]);
% 
% %Basic metrics
% trkWidth = 2*P5(1);
% aKingpin = atand((P3(1)-P4(1))/(P3(2)-P4(2)));
% 
% if P3(1) == P4(1)
%     dScrub = P5(1)-P3(1);
% else
%     dScrub = P5(1) + (P3(2) - ((P3(2)-P4(2))/(P3(1)-P4(1)))*P3(1))/((P3(2)-P4(2))/(P3(1)-P4(1)));
% end
% 
% display("Track width: " + num2str(trkWidth));
% display("Kingpin angle: " + num2str(aKingpin));
% display("Scrub radius: " + num2str(dScrub));
% display("Virtual swing arm length: " + num2str(P5(1)-IC(1)));
% display("Roll center height: " + num2str(RCy));
% display("Camber Gain Rate: " + num2str(atand(1/(P5(1)-IC(1)))));