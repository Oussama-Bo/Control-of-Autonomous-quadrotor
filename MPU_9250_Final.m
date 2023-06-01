% THIS PROGRAM READ GYRO AND ACCELEROMETER VALUES, IT DRAWS THE GRAPHS AS
% WELL
% WORKING FINE
%*****************************************************************************
%%%%%%%%%%%%%%%%%%%%%%%%%% INTRODUCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The MPU9250 contains a die that houses the 3-Axis gyroscope and the 3-Axis 
% accelerometer with a AK8963 3-Axis magnetometer from Asahi Kasei Microdevices
% Corporation. The device works at a VDD range from 2.4V to 3.6V.
% The I²C Module of the MPU9250 supports up to 400kHz clock frequency and it always
% acts as a slave to the Arduino. It also contains an auxiliary I2C master interface
% to talk to the external 3rd party sensor (AK8963).
% The MPU9250 has three 16-bit outputs gyroscope, accelerometer and magnetometer.
% The gyroscope module and accelerometer module feature a user-programmable full-scale 
% range of \pm 250, \pm 500, \pm 1000 and \pm 2000^\circ /sec(dps) and \pm 2g, \pm 4g,
% \pm 8g and \pm 16g respectively. The magnetometer has a fixed full-scale range of \pm 4800\mu T .
% The MPU9250 comes with a digital low-pass filter for the gyroscope and accelerometer.
%*****************************************************************************
clear all;
clc
%*****************************************************************************
a = arduino;% define arduino object
%*****************************************************************************
  mpu9250 = i2cdev(a, '0x68'); % adress the I2C mode slave
  Magneto = i2cdev(a, '0xC');  % adress the I2C of megnetometer
%*****************************************************************************
  
   % Set up and configure MPU
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   writeRegister(mpu9250, hex2dec('6B'), hex2dec('80'), 'uint8'); % Reset the internal registers and restores the default settings. 
   pause(0.1);
   writeRegister(mpu9250, hex2dec('6B'), hex2dec('00'), 'int8'); % or hex2dec('01'), Activate MPU and use Internal 20MHz oscillator
   writeRegister(mpu9250, hex2dec('6C'), hex2dec('00'), 'int8'); % Confirm axes enabled of G and Acc (not necessary)
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Activate Accelerometer
  writeRegister(mpu9250, hex2dec('1C'), hex2dec('00'), 'int8'); % Reset Accelerometer
  writeRegister(mpu9250, hex2dec('1C'), hex2dec('08'), 'int8'); % Accelerometer, Full Scale Select:+/- 4 g
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Activate Gyroscope
  writeRegister(mpu9250, hex2dec('1B'), hex2dec('00'), 'int8'); % reset Gyroscope
  writeRegister(mpu9250, hex2dec('1B'), hex2dec('08'), 'int8'); % Gyroscope Gyro Full Scale Select:+/- 500 dps
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % writeRegister(mpu9250, hex2dec('6B'), hex2dec('00'), 'int8'); % Activate MPU
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % configuration of DLPF: Digital Low Pass filter
 writeRegister(mpu9250, hex2dec('1D'), hex2dec('05'), 'int8');%0000 0101 in binary. Turn on the internal low-pass filter for accelerometer
 % with 10.2Hz bandwidth and 16.83 uS of delay
 writeRegister(mpu9250, hex2dec('1A'), hex2dec('05'), 'int8');%0000 0101 in binary. Turn on the internal low-pass filter for gyroscope
 % with 10Hz bandwidth and 17.5 uS delay
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
 writeRegister(Magneto, hex2dec('0A'), hex2dec('1F'), 'int8'); % setup the Magnetometer to fuse ROM access mode
 % to get the Sensitivity Adjustment values and 16-bit output
 pause(0.1);  %wait for the mode changes
 
 buffer_x = readRegister(Magneto,10,'int8');%read the X Sensitivit Adjustment values
 buffer_y = readRegister(Magneto,11,'int8');%read the Y Sensitivit Adjustment values
 buffer_z = readRegister(Magneto,12,'int8');%read the Z Sensitivit Adjustment values

  asax = ((double(buffer_x)-128)*0.5/128+1);
  asay = ((double(buffer_y)-128)*0.5/128+1);
  asaz = ((double(buffer_z)-128)*0.5/128+1);
  asa=[asax asay asaz];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 writeRegister(Magneto, hex2dec('0A'), hex2dec('00'), 'int8'); % reset the Magnetometer to power down mode (necessary when switch modes)
 pause(0.4);  %wait for the mode changes
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 writeRegister(Magneto, hex2dec('0A'), hex2dec('16'), 'int8'); %set the Magnetometer to continuous mode 2?100Hz) 
 %and 16-bit output
 pause(0.4);  %wait for the mode changes
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Talk to user and pause
  fprintf('Hold IMU still, calibrating...')
  pause(1)
   
  % Set up and configure MPU for +/- 4g and 500 deg/s
  scaleFactorGyro = 65.536; % FS_SEL=01 2^13/500
  scaleFactorAcc = 8192;  %AFS_SEL=01 
  
  % Start counter and timer
  i = 0; % Iteration counter
  tic  % starting time
  timer = toc;  % verification timer
  % Loop through gyro values for 15 seconds
  while toc < 5     % 5 seconds callibration
    i = i + 1;
    g = readGyro(mpu9250, scaleFactorGyro);
    gyroCalX(i) = g.x;
    gyroCalY(i) = g.y;
    gyroCalZ(i) = g.z;
    if (toc - timer) > 0.5
      fprintf('.')   % print a point every half a second
      timer = toc;
    end
  end
  fprintf('\nCalibration complete\n')

  % Calculate average offset
  gyroCal.x = sum(gyroCalX) / length(gyroCalX);
  gyroCal.y = sum(gyroCalY) / length(gyroCalY);
  gyroCal.z = sum(gyroCalZ) / length(gyroCalZ);
  % Display info to user
  fprintf('%0.0f values sampled\n',length(gyroCalX))
  k=1;
 Bx = 26.1325;
By = 15.1875;
Bz = -10.9500;
  Mxo=0;
  Myo=0;
  Mzo=0;
  ref=[0 1 0]; % initialization
  tic
while (toc< 200)
   %**********************************************************************%
   %**********************************************************************%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    statut_1=readRegister(Magneto,2,'uint8'); % statut 1 register
    DRDY = bitget(statut_1,1) ;  %get first bit
  if DRDY ==0
      fprintf('DRDY = 0 No new data')
   else
      fprintf('DRDY = 1 New data Available') 
   end 
  %mag_factor=4912;
  mag_factor=0.15;
  MAG_XOUT_H = readRegister(Magneto,hex2dec('04'),'int8');
  MAG_XOUT_L = readRegister(Magneto,hex2dec('03'),'int8');
  MAG_YOUT_H = readRegister(Magneto,hex2dec('06'),'int8');
  MAG_YOUT_L = readRegister(Magneto,hex2dec('05'),'int8');
  MAG_ZOUT_H = readRegister(Magneto,hex2dec('08'),'int8');
  MAG_ZOUT_L = readRegister(Magneto,hex2dec('07'),'int8');
  reset = readRegister(Magneto,hex2dec('09'),'uint8');% to get new data
  Hofl = bitget(statut_1,4) ;  %get first bit
  if Hofl ==1
      fprintf(' Hofl =1, Bad data')
  end
  Bx = 26.1325;
By = 15.1875;
Bz = -10.9500;
  %calculating values of the acceleration
  Mx = double(typecast(int8([MAG_XOUT_L MAG_XOUT_H]),'int16')) * mag_factor;
  Mx=Mx*asax;
  My = double(typecast(int8([MAG_YOUT_L MAG_YOUT_H]),'int16')) * mag_factor;
  My=My*asay;
  Mz = double(typecast(int8([MAG_ZOUT_L MAG_ZOUT_H]),'int16')) * mag_factor;
  Mz=Mz*asaz;
  %correction of axes (No need)
%   alpha=Mx;
%   Mx=My;
%   My=-alpha;
%   Mz=-Mz;
  Mx=0.7*Mxo+0.3*Mx;
  My=0.7*Myo+0.3*My;
  Mz=0.7*Mzo+0.3*Mz;
  Mxo=Mx;
  Myo=My;
  Mzo=Mz;
   hold on
   quiver3(0,0,0, Mx-Bx,My-By,Mz-Bz,'r','linewidth',3)
   
%  plot3(Mx,My,Mz,'r*');
%  plot3(Mx-Bx,My-By,Mz-Bz,'b*');
  hold on
  grid on
  drawnow
  %**********************************************************************%
  %**********************************************************************%
  % Read Accelerometer
  ACCEL_XOUT_H = readRegister(mpu9250,hex2dec('3B'),'int8');
  ACCEL_XOUT_L = readRegister(mpu9250,hex2dec('3C'),'int8');
  ACCEL_YOUT_H = readRegister(mpu9250,hex2dec('3D'),'int8');
  ACCEL_YOUT_L = readRegister(mpu9250,hex2dec('3E'),'int8');
  ACCEL_ZOUT_H = readRegister(mpu9250,hex2dec('3F'),'int8');
  ACCEL_ZOUT_L = readRegister(mpu9250,hex2dec('40'),'int8');
  %calculating values of the acceleration
  ax = -double(typecast(int8([ACCEL_XOUT_L ACCEL_XOUT_H]),'int16')) / scaleFactorAcc;
  ay = -(double(typecast(int8([ACCEL_YOUT_L ACCEL_YOUT_H]),'int16')) / scaleFactorAcc+1);
  az = -double(typecast(int8([ACCEL_ZOUT_L ACCEL_ZOUT_H]),'int16')) / scaleFactorAcc;
  %**********************************************************************%
  %**********************************************************************%
  % Read Gyroscope
  GYRO_XOUT_H = readRegister(mpu9250,hex2dec('43'),'int8');
  GYRO_XOUT_L = readRegister(mpu9250,hex2dec('44'),'int8');
  GYRO_YOUT_H = readRegister(mpu9250,hex2dec('45'),'int8');
  GYRO_YOUT_L = readRegister(mpu9250,hex2dec('46'),'int8');
  GYRO_ZOUT_H = readRegister(mpu9250,hex2dec('47'),'int8');
  GYRO_ZOUT_L = readRegister(mpu9250,hex2dec('48'),'int8');
  %calculating values of the gyro
  g.x = (double(typecast(int8([GYRO_XOUT_L GYRO_XOUT_H]),'int16')) / scaleFactorGyro) - gyroCal.x;
  g.y = (double(typecast(int8([GYRO_YOUT_L GYRO_YOUT_H]),'int16')) / scaleFactorGyro) - gyroCal.y;
  g.z = (double(typecast(int8([GYRO_ZOUT_L GYRO_ZOUT_H]),'int16')) / scaleFactorGyro) - gyroCal.z;
  %writing in a vector
  gx=g.x; 
  gy=g.y;
  gz=g.z;
     % gyro = [g.x g.y g.z] 
  acceleration =[0 0 -1]; %initialization
  acceleration =[ax ay az]
  gyro=[gx gy gz];
  Magnetic=[0 1 0]; % initialization
  Magnetic=[Mx-Bx My-By Mz-Bz];
  quiver3(0,0,0, 10*ax,10*ay,10*az,'b','linewidth',3)
  drawnow
  hold on

  if k==30
      NN=cross(acceleration,Magnetic);
  ref=cross(NN,acceleration);
     
  quiver3(0,0,0, 2*ref(1),2*ref(2),2*ref(3),'m','linewidth',4)
  drawnow
  hold on
  end
  global theta phi psi
  theta = atan (ay/az)*180/3.14;
  phi= atan(-ax/(sqrt(ay^2+az^2)))*180/3.14;
  NN=cross(acceleration,Magnetic);
  quiver3(0,0,0, NN(1),NN(2),NN(3),'g','linewidth',2)
  drawnow
  hold on
  northmag=cross(NN,acceleration);
  psi=atan2d(norm(cross(northmag,ref)),dot(northmag,ref));
   k=k+1;
   quiver3(0,0,0, northmag(1),northmag(2),northmag(3),'Y','linewidth',3)
  drawnow
  hold off
   
end