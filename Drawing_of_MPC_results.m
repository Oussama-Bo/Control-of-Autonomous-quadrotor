load('X3Y3Z3mpc.mat')
plot3(X3Y3Z3mpc(1:22000,2),X3Y3Z3mpc(1:22000,5),X3Y3Z3mpc(1:22000,8)+0.8,'r','linewidth',2.5)
hold on
plot3(xdata(:,3),ydata(:,3),zdata(:,3),'b','linewidth',1.8)
hold on
grid on
plot3(LPx(:,2)-0.2,LPy(:,2),LPz(:,2),'g','linewidth',1.8)
hold on
plot3(X3Y3Z3(:,2),X3Y3Z3(:,5),X3Y3Z3(:,8),'m','linewidth',2)
legend('PID tracking trajectory','LQG tracked trajectory','Min Jerk desired trajectory','Landing pad trajectory')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


