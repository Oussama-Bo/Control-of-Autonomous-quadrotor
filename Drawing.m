% open the model
% print('-s modelname','-dtiff','model.tiff')
figure(1)
plot(phi_ang(:,1),phi_ang(:,3),'r','linewidth',1.5)
hold on; plot(phi_ang(:,1),phi_ang(:,2),'b','linewidth',1.5)
title('Phi tracking')
xlabel('Time, (sec)')
ylabel('phi & phi_d, (rad)')
legend ('phi desired','phi quadrotor ')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2)
plot(theta_ang(:,1),theta_ang(:,3),'r','linewidth',1.5)
hold on; plot(theta_ang(:,1),theta_ang(:,2),'b','linewidth',1.5)
title('theta tracking')
xlabel('Time, (sec)')
ylabel('theta & theta_d, (rad)')
legend ('theta desired','theta quadrotor ')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(3)

plot(ksi_ang(:,1),ksi_ang(:,2),'b','linewidth',0.2);hold on;
plot(ksi_ang(:,1),ksi_ang(:,3),'r','linewidth',2.5)
title('Psi tracking')
xlabel('Time, (sec)')
ylabel('Psi & Psi_d, (rad)')
legend ('psi desired','psi quadrotor ')
grid on



figure(4)

plot(xdata(:,1),xdata(:,3),'r','linewidth',1.5)
hold on; plot(xdata(:,1),xdata(:,2),'b','linewidth',1.5)
title('X position tracking')
xlabel('Time, (sec)')
ylabel('X & X_d, (m)')
legend ('X desired','X quadrotor ')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5)

plot(ydata(:,1),ydata(:,3),'r','linewidth',1.5)
hold on; plot(ydata(:,1),ydata(:,2),'b','linewidth',1.5)
title('Y position tracking')
xlabel('Time, (sec)')
ylabel('Y & Y_d, (m)')
legend ('Y desired','Y quadrotor ')
grid on

%%%%%%%%%%%%%%%%%%%%%%%%
figure(6)

plot(zdata(:,1),zdata(:,2),'r','linewidth',1.8)
hold on; plot(zdata(:,1),zdata(:,3),'b','linewidth',1.1)
title('Z position tracking')
xlabel('Time, (sec)')
ylabel('Z & Z_d, (m)')
legend ('Z quadrotor ','Z desired')
grid on

figure(7)


plot3(xdata(:,2),ydata(:,2),zdata(:,2),'r','linewidth',1.8)
hold on;
plot3(xdata(:,3),ydata(:,3),zdata(:,3),'b','linewidth',1.8)
title('3D position tracking of the quadrotor')
xlabel('X , (m)')
ylabel('Y, (m)')
zlabel('Z, (m)')
legend ('Quadrotor position ','Desired trajectory')
plot3 (0,0,0,'linewidth',4)
plot3 (0,0,0,'r*','linewidth',8)
grid on


plot3(xdata(:,2),ydata(:,2),zdata(:,2),'r','linewidth',1.8)
hold on;
plot3(xdata(:,3),ydata(:,3),zdata(:,3),'b','linewidth',1.8)
title('3D position tracking of the quadrotor')
xlabel('X , (m)')
ylabel('Y, (m)')
zlabel('Z, (m)')
legend ('Quadrotor position ','Desired trajectory')
plot3 (0,0,0,'linewidth',4)
plot3 (0,0,0,'r*','linewidth',8)
grid on


