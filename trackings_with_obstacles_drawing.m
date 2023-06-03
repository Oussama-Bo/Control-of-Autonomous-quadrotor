
%bring data of FLC
load('avoidanceFLC.mat')
figure 
plot3(xdata2(:,3)-4,ydata2(:,2)-4,zdata2(:,2),'c','linewidth',1.5)
hold on

%bring data of IBS
load('avoidanceBSP.mat');
[xref]=[xdata2(:,3)];
[yref]=[ydata2(:,3)];
[zref]=[zdata2(:,3)];
plot3(xref-4,yref-4,zref,'b','linewidth',1.5)
hold on
plot3(xdata2(:,2)-4,ydata2(:,2)-4,zdata2(:,2),'m','linewidth',1.5)
% Bring data of PID
load('avoidancePID.mat')
hold on
plot3(X3Y3Z3(:,3)-4,X3Y3Z3(:,6)-4,X3Y3Z3(:,9),'g','linewidth',1.5)
% Bring data of LQT
load('avoidanceLQT.mat')
plot3(X3Y3Z3(:,3)-4,X3Y3Z3(:,6)-4,X3Y3Z3(:,9)+0.7,'r','linewidth',1.5);
hold on;
grid on
title('Tracking of Spline trajectory for Avoidance of obstacles ')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend ('FLC Tracking','Desired Spline trajectory','IBS control','PID control','LQT control')
[hleg1, hobj1]=legend('FLC Tracking','Desired Spline trajectory','IBS control','PID control','LQT control');
textobj = findobj(hobj1,'type','text');
set(textobj,'Interpreter','latex','fontsize',15)
set(hleg1,'position',[0.7 0.6 0.25 0.25])


%################################################################
%################################################################
%################################################################
% THIS PROGRAM DRAWS THE BLOCK TO BE AVOIDED THEN IT TRACES THE TRAJECTORY
% FOR THE QUADROTOR FOLLOWING A SPLINE TRAJ.
%################################################################
%################################################################
%################################################################



a = -pi : pi/2 : pi;  % Define Corners
ph = pi/4;  % Define Angular Orientatio
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%first cube
x = [cos(a+ph); cos(a+ph)]/cos(ph)/3;
y = [1+sin(a+ph); 1+sin(a+ph)]/sin(ph);
z = [-ones(size(a)); ones(size(a))];
%figure
surf(x, y, z, 'FaceColor','g')  % Plot Cube
%surf(x, y, z)  % Plot Cube
hold on
patch(x', y', z', 'g')  % Make Cube Appear Solid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%Second cube
x1 = [cos(a+ph); cos(a+ph)]/cos(ph);
y1 = [7+sin(a+ph); 7+sin(a+ph)]/sin(ph);
z1 = [-ones(size(a)); ones(size(a))];


%surf(x, y, z, 'FaceColor','b')  % Plot Cube
surf(x1, y1, z1,'FaceColor','r')  % Plot Cube
hold on
patch(x1', y1', z1', 'r')  % Make Cube Appear Solid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Third cube
x2 = [cos(a+ph)-2; cos(a+ph)]/(cos(ph)*0.4);
y2 = [4+sin(a+ph); 4+sin(a+ph)]/sin(ph);
z2 = [-ones(size(a)); ones(size(a))];

%surf(x, y, z, 'FaceColor','b')  % Plot Cube
surf(x2, y2, z2)  % Plot Cube
hold on
patch(x2', y2', z2', 'b')  % Make Cube Appear Solid

plot3(-4,-4,0,'ro')

plot3(-4,10,0.2,'r*')
xlabel('x')
ylabel('y')
zlabel('z')


axis([ -10 10 -10 10 -1 2])
grid on