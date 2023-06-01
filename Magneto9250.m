clear all
clc
%*****************************************************************************
%%%%%%%%%%%%%%%%%%%%%%%%%% INTRODUCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The MPU9250 contains an external 3rd party sensor (AK8963)
% The magnetometer has a fixed full-scale range of \pm 4800\mu T .
%*****************************************************************************
a = arduino;% define arduino object
scanI2CBus(a);
%*****************************************************************************
  mpu9250 = i2cdev(a, '0x68'); % adress the I2C mode slave
  Magneto = i2cdev(a, '0x0C');

  %Magneto2 = i2cdev(a, '0xC');
%*****************************************************************************
  
   % Set up and configure MPU
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   writeRegister(mpu9250, hex2dec('6B'), hex2dec('80'), 'uint8'); % Reset the internal registers and restores the default settings. 
   pause(0.1);
   writeRegister(mpu9250, hex2dec('6B'), hex2dec('00'), 'int8'); % or hex2dec('01'), Activate MPU and use Internal 20MHz oscillator
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % the MPU 9250 communicated with third party magnetometer  AK8963 as a I2C
 % master via an Interface ByPass master.
 % The auxiliary I2C master interface of the MPU9250 contains 
 % an interface bypass multiplexer, which allows the system processor program the 3rd party sensor.
 % The bypass multiplexer is controlled by the register 55
 % turn on the bypass multiplexer
 writeRegister(mpu9250, hex2dec('6A'), hex2dec('00'), 'int8'); % disable i2C inteface first 
 writeRegister(mpu9250, hex2dec('37'), hex2dec('02'), 'int8'); %0000 0010 in binary, turn on the bypass multiplexer
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % change the reading mode of data
 writeRegister(Magneto, hex2dec('0A'), hex2dec('0F'), 'int8'); % setup the Magnetometer to fuse ROM access mode
 % to get the Sensitivity Adjustment values and 16-bit output
 pause(0.1);  %wait for the mode changes
 buffer_x = readRegister(Magneto,10,'uint8');%read the X Sensitivit Adjustment values
 buffer_y = readRegister(Magneto,11,'uint8');%read the Y Sensitivit Adjustment values
 buffer_z = readRegister(Magneto,12,'uint8');%read the Z Sensitivit Adjustment values
  asax = ((double(buffer_x)-128)*0.5/128+1);
  asay = ((double(buffer_y)-128)*0.5/128+1);
  asaz = ((double(buffer_z)-128)*0.5/128+1);
  asa=[asax asay asaz];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 writeRegister(Magneto, hex2dec('0A'), hex2dec('00'), 'int8'); % reset the Magnetometer to power down mode
 pause(0.4);  %wait for the mode changes
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 writeRegister(Magneto, hex2dec('0A'), hex2dec('16'), 'int8'); %set the Magnetometer to continuous mode 2?100Hz) 
 %and 16-bit output
 pause(0.4);  %wait for the mode changes
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%   axl = animatedline('Color',[1 0 0]); 
%   ayl = animatedline('Color',[0 1 0]); 
%   azl = animatedline('Color',[0 0 1]); 
% 
%   gxl = animatedline('Color',[1 1 0]); 
%   gyl = animatedline('Color',[0 1 1]); 
%   gzl = animatedline('Color',[1 0 1]);
%   
%   legend('Accel_x','Accel_y','Accel_z','Gyro_x','Gyro_y','Gyro_z')

  k=1;
  tic
    axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
Bx = 26.1325;
By = 15.1875;
Bz = -10.9500;
while (toc< 150)
   statut_1=readRegister(Magneto,2,'uint8'); % statut 1 register
    DRDY = bitget(statut_1,1) ;  %get first bit
   if DRDY ==0
      fprintf('DRDY = 0 No new data')
   else
      fprintf('DRDY = 1 New data Available') 
   end 
  mag_factor=0.15;
  MAG_XOUT_H = readRegister(Magneto,hex2dec('04'),'int8');
  MAG_XOUT_L = readRegister(Magneto,hex2dec('03'),'int8');
  MAG_YOUT_H = readRegister(Magneto,hex2dec('06'),'int8');
  MAG_YOUT_L = readRegister(Magneto,hex2dec('05'),'int8');
  MAG_ZOUT_H = readRegister(Magneto,hex2dec('08'),'int8');
  MAG_ZOUT_L = readRegister(Magneto,hex2dec('07'),'int8');
   reset = readRegister(Magneto,hex2dec('09'),'int8');% to get new data
  Hofl = bitget(statut_1,4) ;  %get first bit
   if Hofl ==1
       fprintf(' Hofl =1, Bad data')
   end
  
  %calculating values of the acceleration
  Mx = double(typecast(int8([MAG_XOUT_L MAG_XOUT_H]),'int16')) * mag_factor;
  Mx=Mx*asax;
  My = double(typecast(int8([MAG_YOUT_L MAG_YOUT_H]),'int16')) * mag_factor;
  My=My*asay;
  Mz = double(typecast(int8([MAG_ZOUT_L MAG_ZOUT_H]),'int16')) * mag_factor;
  Mz=Mz*asaz;
  %correction of axes
%   alpha=Mx;
%   Mx=My;
%   My=alpha;
%   Mz=-Mz;
  M(:,k)=[Mx ;My; Mz];
%   M=sqrt(Mx^2+My^2+Mz^2);
%   MxN=Mx/M;
%   MyN=My/M;
%   MzN=Mz/M;
 % quiver3(0,0,0, Mx-Bx,My-By,Mz-Bz)
 % axis equal
 plot3(Mx,My,Mz,'r*');
 plot3(Mx-Bx,My-By,Mz-Bz,'b*');
 % axis([-20 20 -20 20 -20 20])
 %quiver3(0,0,0, Mx-Bx,My-By,Mz-Bz)
 hold on
 grid on
 drawnow
  %  end
    k=k+1;
  %pause(0.1)
  GG=[0 0 -1];
  if k==3
  NN=cross(GG,[Mx-Bx My-By Mz-Bz]);
  ref=cross(NN,GG);
     end
  %theta = atan (ay/az)*180/3.14
  %phi= atan(-ax/(sqrt(ay^2+az^2)))*180/3.14
  NN=cross(GG,[Mx-Bx My-By Mz-Bz]);
  northmag=cross(NN,GG);
  psi=atan2d(norm(cross(northmag,ref)),dot(northmag,ref))
  
end
Bx=(max(M(1,:))+min(M(1,:)))/2
By=(max(M(2,:))+min(M(2,:)))/2
Bz=(max(M(3,:))+min(M(3,:)))/2
hold on; quiver3(0,0,0, 5,0,0,'m','linewidth',3)
hold on; quiver3(0,0,0, 0,5,0,'m','linewidth',3)
hold on; quiver3(0,0,0, 0,0,5,'m','linewidth',3)
xlabel('X values')
ylabel('y values')
zlabel('Z values')
title('Calibration of Magnetometer in MPU9250')
