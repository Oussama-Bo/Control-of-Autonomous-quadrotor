

%     PROGRAM OF 3D SPLINE THAT GENERATES THE TRAJECTORY BASED ON DESIRED
%     COORDONATES, THE PROGRAM WORKS PERFECTLY AND HAS SAME RESULTS AS
%     MATLAB FUNCTION



%####################################################
% DEFINITION OF POINTS
%####################################################
t = [1,2,3,4,5,6,7,8,9];  
t=t*3;
x=[-4 -2 0 3 4 3 0 -2 -4 ];
y=[-4 2 3 3 6 8 8 7 10];
z=[0 0 0 0.3 0.5 0.2 0 0 0.2];
%####################################################
% DrAWING OF POINTS
%####################################################
n=length (t)
for i=1:n
    plot3(x(i),y(i),z(i),'r*')
    hold on
end
grid on
%####################################################
% First method
%####################################################
tt = linspace(t(1),t(end));
xx = interp1(t,x,tt,'spline');
yy = interp1(t,y,tt,'spline');
zz = interp1(t,z,tt,'spline');
% Visualize the result
scatter3(x,y,z)
hold on
plot3(xx,yy,zz,'linewidth',1)
%####################################################
% SECOND METHOD
%####################################################
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:3
s(i,1)=0;
s(i,n)=0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for j=2:n-1
    s(1,j)=(x(j+1)-x(j-1))/(t(j+1)-t(j-1));
    s(2,j)=(y(j+1)-y(j-1))/(t(j+1)-t(j-1));
    s(3,j)=(z(j+1)-z(j-1))/(t(j+1)-t(j-1));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:n-1
    syms a b c d
        eqt1=a*t(i)^3+b*t(i)^2+c*t(i)+d==x(i);
        eqt2=a*t(i+1)^3+b*t(i+1)^2+c*t(i+1)+d==x(i+1);
        eqt3=3*a*t(i)^2+2*b*t(i)+c==s(1,i);
        eqt4=3*a*t(i+1)^2+2*b*t(i+1)+c==s(1,i+1);
[a,b,c,d]= solve ([eqt1,eqt2,eqt3,eqt4], [a,b,c,d]);        
Sol = [a,b,c,d];
Sol= double (Sol);;
aa(1,i)=a;
bb(1,i)=b;
cc(1,i)=c;
dd(1,i)=d;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:n-1
    syms a b c d
        eqt1=a*t(i)^3+b*t(i)^2+c*t(i)+d==y(i);
        eqt2=a*t(i+1)^3+b*t(i+1)^2+c*t(i+1)+d==y(i+1);
        eqt3=3*a*t(i)^2+2*b*t(i)+c==s(2,i);
        eqt4=3*a*t(i+1)^2+2*b*t(i+1)+c==s(2,i+1);
[a,b,c,d]= solve ([eqt1,eqt2,eqt3,eqt4], [a,b,c,d]);        
Sol = [a,b,c,d];
Sol= double (Sol);;
aa(2,i)=a;
bb(2,i)=b;
cc(2,i)=c;
dd(2,i)=d;
end

for i=1:n-1
    syms a b c d
        eqt1=a*t(i)^3+b*t(i)^2+c*t(i)+d==z(i);
        eqt2=a*t(i+1)^3+b*t(i+1)^2+c*t(i+1)+d==z(i+1);
        eqt3=3*a*t(i)^2+2*b*t(i)+c==s(3,i);
        eqt4=3*a*t(i+1)^2+2*b*t(i+1)+c==s(3,i+1);
[a,b,c,d]= solve ([eqt1,eqt2,eqt3,eqt4], [a,b,c,d]);        
Sol = [a,b,c,d];
Sol= double (Sol);;
aa(3,i)=a;
bb(3,i)=b;
cc(3,i)=c;
dd(3,i)=d;
end
pause(5);
for i =1:n-1
   
tt(i,:)= linspace (t(i),t(i+1));              % define the intervals every time
hold on
grid on
plot3 (aa(1,i)*tt(i,:).^3+bb(1,i)*tt(i,:).^2+cc(1,i)*tt(i,:)+dd(1,i),aa(2,i)*tt(i,:).^3+bb(2,i)*tt(i,:).^2+cc(2,i)*tt(i,:)+dd(2,i),aa(3,i)*tt(i,:).^3+bb(3,i)*tt(i,:).^2+cc(3,i)*tt(i,:)+dd(3,i),'bo')  % plot of the SPLINE polynomial
 pause(0.5)       
end    





