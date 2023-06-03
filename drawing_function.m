load('NN_results2.mat')
plot3(xdata(:,3),ydata(:,3),zdata(:,3),'r','linewidth',2)
hold on
plot3(xdata(:,2),ydata(:,2),zdata(:,2),'m','linewidth',1.2)
grid on
hold on
aa =length (xdata);
load('RBF_results.mat');
xdata=imresize (xdata, [aa 3] );
ydata=imresize (ydata, [aa 3] );
zdata=imresize (zdata, [aa 3] );
plot3(xdata(:,2),ydata(:,2),zdata(:,2),'b','linewidth',2)
load('IBS_results.mat');
xdata=imresize (xdata, [aa 3] );
ydata=imresize (ydata, [aa 3] );
zdata=imresize (zdata, [aa 3] );
plot3(xdata(:,2),ydata(:,2),zdata(:,2),'g','linewidth',1.2)

load('PID_results.mat');
xdata=imresize (xdata, [aa 3] );
ydata=imresize (ydata, [aa 3] );
zdata=imresize (zdata, [aa 3] );
plot3(xdata(:,2),ydata(:,2),zdata(:,2),'linewidth',1.2)

title('Adaptive RBFNN Position+ SUPERVISED RBF-PID Attitude CONTROL for Tracking in comparison with MLP NN control, IBS control and PID');
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
legend ('Desired Trajectory to be tracked','Quadrotor simulated MLP NN trajectory Tracking','Quadrotor simulated trajectory with proposed Adaptive RBFNN control','Quadrotor IBS control','Quadrotor PID control')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
clc
load('RBF_results.mat')
figure (2)
subplot(3,1,1)
plot(xdata(:,1),xdata(:,2),'r','linewidth',1)
hold on
plot(xdata(:,1),xdata(:,3),'b','linewidth',1)
grid on
hold on
xlabel('time(sec)')
ylabel('X(m)')
legend ('X_{quadrotor}','X_{desired}')

subplot(3,1,2)
plot(ydata(:,1),ydata(:,2),'r','linewidth',1)
hold on
plot(ydata(:,1),ydata(:,3),'b','linewidth',1)
grid on
hold on
xlabel('time(sec)')
ylabel('Y(m)')
legend ('Y_{quadrotor}','Y_{desired}')

subplot(3,1,3)
plot(zdata(:,1),zdata(:,2),'r','linewidth',1)
hold on
plot(zdata(:,1),zdata(:,3),'b','linewidth',1)
grid on
hold on
xlabel('time(sec)')
ylabel('Z(m)')
legend ('Z_{quadrotor}','Z_{desired}')
figure (2)
title('The translation tracking responses of the quadrotor aircraft; Axes: X, Y, and Z');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure (3)
subplot(3,1,1)
plot(phi(:,1),phi(:,2),'r','linewidth',1)
hold on
plot(phi(:,1),phi(:,3),'b','linewidth',1)
grid on
hold on
xlabel('time(sec)')
ylabel('Phi(rad)')
legend ('Phi','Phi_{des}')


subplot(3,1,2)
plot(theta(:,1),theta(:,2),'r','linewidth',1)
hold on
plot(theta(:,1),theta(:,3),'b','linewidth',1)
grid on
hold on
xlabel('time(sec)')
ylabel('Theta(rad)')
legend ('Theta','Theta_{des}')

subplot(3,1,3)
plot(psi(:,1),psi(:,2),'r','linewidth',1)
hold on
plot(psi(:,1),psi(:,3),'b','linewidth',1)
grid on
hold on
xlabel('time(sec)')
ylabel('Psi(rad)')
legend ('Psi','Psi_{des}')

figure (3);
title('The attitude tracking responses of the quadrotor aircraft; angles: Phi, Theta, and Psi');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
clc
load('RBF_results.mat')

figure (4);
title('The position and attitude errors of the quadrotor tracking of the desired trajectpory');
subplot(2,1,1)
plot(xdata(:,1),xdata(:,2)-xdata(:,3),'b','linewidth',1)
grid on
hold on

subplot(2,1,1)
plot(ydata(:,1),ydata(:,2)-ydata(:,3),'r','linewidth',1)
grid on
hold on

subplot(2,1,1)
plot(zdata(:,1),zdata(:,2)-zdata(:,3),'g','linewidth',1)
grid on
hold on
ylabel('Error x,y,z(m)');
legend ('X_{error}','Y_{error}','Z_{error}');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


subplot(2,1,2)
plot(phi(:,1),(phi(:,2)-phi(:,3))/2,'b','linewidth',1)
grid on
hold on

subplot(2,1,2)
plot(theta(:,1),(theta(:,2)-theta(:,3))/2,'r','linewidth',1)
grid on
hold on


subplot(2,1,2)
plot(psi(:,1),(psi(:,2)-psi(:,3))/2,'g','linewidth',1)
grid on
hold on

xlabel('Time(sec)')
ylabel('Error Phi,Theta,Psi(rad)')
legend ('Phi_{error}','Theta_{error}','Psi_{error}')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%    %    %
 % %  %  %  
  %    %
load('NN_results2.mat')
figure (5)
subplot(3,1,1)
for i=1:5
plot(w(:,1),w(:,i+1),'linewidth',1)
hold on
grid on
end
%xlabel('time(sec)');
ylabel('W(real)');
legend ('w1','w2','w3','w4','w5');
title('W_i parameters of weights convergence ( one of the six RBF networks)');

subplot(3,1,2)
for i=1:5
plot(b(:,1),b(:,i+1),'linewidth',1)
hold on
grid on
end
%xlabel('time(sec)');
ylabel('B(real)');
legend ('b1','b2','b3','b4','b5');
title('b_i parameters of width convergence (one of the six RBF networks)');

subplot(3,1,3)
for i=1:5
plot(c(:,1),c(:,i+1),'linewidth',1)
hold on
grid on
end
xlabel('time(sec)');
ylabel('C(real)');
legend ('c1','c2','c3','c4','c5');
title('c_i parameters of centers convergence (one of the six RBF networks)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6)
plot(Dist(:,1),Dist(:,5),'r','linewidth',1);
hold on
plot(Dist(:,1),Dist(:,6),'b','linewidth',1);
hold on
plot(Dist(:,1),Dist(:,7),'g','linewidth',1);
hold on;
grid on;
xlabel('time(sec)')
ylabel('Disturbance estimation(Signal)')
legend ('D_x','D_y','D_z')
title('Adaptive RBF network disturbance estimation');  