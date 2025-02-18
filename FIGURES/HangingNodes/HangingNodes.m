clear all, close all
% P1 = [0,0];
% P2 = [1,0];
% P3 = [1,1];
% P4 = [0,1];
% 
% P5 = [1,0.5];
% P6 = [1.5,0];
% P7 = [1.5,0.5];
% P8 = [1.5,1];

% figure(1)
% plot([P1(1);P2(1);P3(1);P4(1);P1(1)],[P1(2);P2(2);P3(2);P4(2);P1(2)],'.k','MarkerSize',35,'LineWidth',2)
% hold on
% plot([P2(1);P6(1);P7(1);P5(1);P2(1)],[P2(2);P6(2);P7(2);P5(2);P2(2)],'.k','MarkerSize',35,'LineWidth',2)
% plot([P3(1);P5(1);P7(1);P8(1);P3(1)],[P3(2);P5(2);P7(2);P8(2);P3(2)],'.k','MarkerSize',35,'LineWidth',2)

%%

P1 =[0,0];
P2 =1/2*[-1,1];
P3 = [0,1];
P4 = 1/2*[1,1];
P5 = 1/2*[0,1];
l = 0.38
figure(2)
plot([P1(1);P2(1);P3(1);P1(1)],[P1(2);P2(2);P3(2);P1(2)],'.-k','MarkerSize',35,'LineWidth',2)
hold on
plot([P1(1);P4(1);P5(1);P1(1)],[P1(2);P4(2);P5(2);P1(2)],'.-k','MarkerSize',35,'LineWidth',2)
plot([P3(1);P5(1);P4(1);P3(1)],[P3(2);P5(2);P4(2);P3(2)],'.-k','MarkerSize',35,'LineWidth',2)
quiver(P5(1)-l,P5(2)+l,l,-l,'r','LineWidth',2,'MarkerSize',35,'MaxHeadSize',10)
axis equal
axis off