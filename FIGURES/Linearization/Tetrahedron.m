clear all, close all
P1 = [0,0,0];
P2 = 1 * [0, sin(pi/6),cos(pi/6)];
P3 = 1 * [0,-sin(pi/6),cos(pi/6)];
P4 = 1 * [sin(pi/6),0,2*cos(pi/6)/3];
P5 = 1 * [-sin(pi/6),0,2*cos(pi/6)/3];

V1 = 0;
V2 = 0.35;
V3 = 0.4;
V4 = 0.15;
%V5 = 0.15;


PM1 = 1/4*(P1+2*P2+P3);
PM2 = 1/4*(P1+P3+2*P4);

r = 0.04;
omega = 12*pi;
dir1 = (PM2-PM1);
dir1 = dir1/norm(dir1);
dir2 = P3-P1;
dir2 = dir2/norm(dir2);
dir3 = cross(dir1,dir2);
dir3 = dir3/norm(dir3);
f = @(s) PM1 + (PM2-PM1)*s +r*(dir2*cos(omega*s)+dir3*sin(omega*s))   ;%s = [0,1]

g1 = [P1;P2;P3;P4]\[V1;V2;V3;V4]%of right tetrahedron
% g2 = [P1;P2;P3;P5]\[V1;V2;V3;V5]

MaSi=40;


figure(1)
plot3([P1(1);P2(1)],[P1(2);P2(2)],[P1(3);P2(3)],'-k','MarkerSize',MaSi,'LineWidth',2)
hold on
plot3([P2(1);P3(1)],[P2(2);P3(2)],[P2(3);P3(3)],'.-k','MarkerSize',MaSi,'LineWidth',2)
plot3([P3(1);P1(1)],[P3(2);P1(2)],[P3(3);P1(3)],'.--k','MarkerSize',MaSi,'LineWidth',2)
plot3([P1(1);P4(1)],[P1(2);P4(2)],[P1(3);P4(3)],'.-k','MarkerSize',MaSi,'LineWidth',2)
plot3([P2(1);P4(1)],[P2(2);P4(2)],[P2(3);P4(3)],'.-k','MarkerSize',MaSi,'LineWidth',2)
plot3([P3(1);P4(1)],[P3(2);P4(2)],[P3(3);P4(3)],'.-k','MarkerSize',MaSi,'LineWidth',2)
plot3(PM1(1),PM1(2),PM1(3),'ro','MarkerSize',8,'MarkerFaceColor','r')
plot3(PM2(1),PM2(2),PM2(3),'ro','MarkerSize',8,'MarkerFaceColor','r')
plot3([PM1(1);PM2(1)],[PM1(2);PM2(2)],[PM1(3);PM2(3)],'r--','MarkerSize',8,'MarkerFaceColor','r','LineWidth',2)

%plot gyro motion:
s_vec = linspace(0,1,1000).';
f_vec = nan(numel(s_vec),3);
for i = 1:numel(s_vec);
    f_vec(i,:) = f(s_vec(i));
end

plot3(f_vec(:,1),f_vec(:,2),f_vec(:,3),'-r','LineWidth',1)
pos_arrow = f_vec(end,:);
dir_arrow = 15*(f_vec(end,:)-f_vec(end-1,:));
quiver3(pos_arrow(1),pos_arrow(2),pos_arrow(3),dir_arrow(1),dir_arrow(2),dir_arrow(3),'r')

% quiver3(0,0,0,g1(1),g1(2),g1(3),'-r','LineWidth',2)
%
% plot3([P1(1);P5(1)],[P1(2);P5(2)],[P1(3);P5(3)],'.-k','MarkerSize',30,'LineWidth',2)
% plot3([P2(1);P5(1)],[P2(2);P5(2)],[P2(3);P5(3)],'.-k','MarkerSize',30,'LineWidth',2)
% plot3([P3(1);P5(1)],[P3(2);P5(2)],[P3(3);P5(3)],'.-k','MarkerSize',30,'LineWidth',2)
% quiver3(0,0,0,g2(1),g2(2),g2(3),'-b','LineWidth',2)

% P = [P1;P2;P3;P4;P5];
% for i=1:5
%     text(P(i,1)+0.01,P(i,2),P(i,3)-0.015,['  ',num2str(i)],'FontSize',32,'Color','k')
% end
x = [P1(1),P2(1),P3(1)];
y = [P1(2),P2(2),P3(2)];
z = [P1(3),P2(3),P3(3)];
 patch(x,y,z,'blue','FaceAlpha',0.1)
x = [P1(1),P3(1),P4(1)];
y = [P1(2),P3(2),P4(2)];
z = [P1(3),P3(3),P4(3)];
 patch(x,y,z,'red','FaceAlpha',0.1)
axis equal
axis off
view(150.34,4.8427)