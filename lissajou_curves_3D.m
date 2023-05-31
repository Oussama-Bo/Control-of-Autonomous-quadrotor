t=0:0.2:30*pi;
A=5;a=5;
B=10;b=4;
C=8;c=5;
x=A*sin(a*0.070*t+pi/2);
y=B*sin(b*0.07*t);
z=C*(1-exp(-5*0.070*t));
plot3(x,y,z,'b','linewidth',2)
title('Panoramic view of desired traj')
xlabel('X, (m)')
ylabel('Y , (m)')
zlabel('Z , (m)')
%legend ('phi desired','phi quadrotor ')
grid on