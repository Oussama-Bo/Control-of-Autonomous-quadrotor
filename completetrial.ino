#include <Wire.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double gyroX=0, gyroY=0, gyroZ=0;                  // for reading gyro data
double gyroXc=0, gyroYc=0, gyroZc=0;                 // for gyro calibration
double rotXn=0,rotYn=0,rotZn=0,rotXo=0,rotYo=0,rotZo=0;     // gyro values in deg/sec
byte highByte, lowByte;                        // two read 16 bits (2 bytes) data
int cal_int;                                      // for gyro's calibration incrementer 
int rotX=0, rotY=0, rotZ=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int alpha;                               // not used
int state;                             // state of the machine's control
int b,tt;                              // variable for led and for ESC's calibration
//Declaring Variables
byte last_channel_1,last_channel_2,last_channel_3,last_channel_4,last_channel_5;                     // last chanels state to detect the up or down
int receiver_input_channel_1,receiver_input_channel_2,receiver_input_channel_3=0,receiver_input_channel_4=0,receiver_input_channel_5 ;      // receiver inputs 
int start, throttle;                                                                                          // internal variable
unsigned long timer_mot_1,timer_mot_2,timer_mot_3,timer_mot_4,timer_mot;                                  // signals for motors and end of timers varibale
unsigned long esc_timer, esc_loop_timer;                                                                  // internal variable for motors timers
unsigned long zero_timer, timer_1,timer_2,timer_3,timer_4,timer_5, current_time;                          // counters from the receiver subroutine 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float k;
float r_p_gain = 3.2;               //Gain of proportional controller for roll 5
float r_i_gain = 0.01;              //Gain of integral controller for roll  0.01
float r_d_gain = 12;                //Gain of derivative controller for roll   19.9
int r_max = 350, correction;                    //Maximum permissible value of the roll PID-controller

float p_p_gain = r_p_gain;         //Gain of proportional controller for pitch
float p_i_gain = r_i_gain;  //Gain of integral controller for pitch
float p_d_gain = r_d_gain;  //Gain of derivative controller for pitch
int p_max = 350;          //Maximum permissible value of the pitch PID-controller

float y_p_gain = 0.5;                //Gain of proportional controller for Yaw
float y_i_gain = 0.01;               //Gain of integral controller for Yaw
float y_d_gain = 0;                //Gain of derivative controller for yaw 2
int y_max = 250;                     //Maximum permissible value of the yaw PID-controller

float r_error, y_error, p_error;                                       // errors of RPY
float i_roll, r_setpoint,  pid_r, last_r_d_error;                  // PID coeficients for R
float i_pitch, p_setpoint,  pid_p, last_p_d_error;           // PID coeficients for P
float i_yaw, y_setpoint,  pid_y, last_y_d_error;             // PID coeficients for Y


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  Wire.begin();
  Serial.begin(9600);
   Serial.println("uploading data ...");   

i_roll=0;
i_pitch=0;                                        // PID reseting
i_yaw=0;
last_r_d_error=0;
last_p_d_error=0;
last_y_d_error=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  state =0;
  start =0;                                          // state definition at start
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  DDRD |= B11111100;                                 //Configure digital port 2.3. 4 5 6 7 as output
  
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT"i" (digital input 8-9-10-11-12-13) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // 
  PCMSK0 |= (1 << PCINT2); 
  PCMSK0 |= (1 << PCINT3);
  PCMSK0 |= (1 << PCINT4);
   Serial.println("Activated Pin change interrupt ");   
delay (500);
   Serial.println("Calibration of ESCs ... in progress... ");   

ESC_calibration(); 
delay (1000);
setup_gyro();                                              // start communication with gyro, I2C adressing and data update bloackage 
  Serial.println("Calibration  of gyro ... in progress...");              //Print message
    delay (300);       // wait for gyro to respond

  // calibration
  for (cal_int = 0; cal_int < 1800 ; cal_int ++){          //Take 1800 readings for calibration
    gyroXc  += gyroX;                                       //Ad roll value to gyroXc
    gyroYc  += gyroY;                                      //Ad pitch value to gyroZc
    gyroZc  += gyroZ;                                      //Ad yaw value to gyroYc
    recordGyroRegisters();                                 //Read the gyro output 1800 times by for loop
     Serial.println(cal_int);                               
    stop_bipping ()     ;             // some 1000 micor sec signals to motors                        
        if(cal_int %100 == 0){                              
       digitalWrite(2,b);  // led to understand that we wait for starting
       b=!b;                                 
       }
  }
  //Now that we have 1800 measures, we need to devide by 2000 to get the average gyro offset
      
  gyroXc /= 1800;               // those values will be substructed from the future readings in the program control loop                          
  gyroYc /= 1800;                                          
  gyroZc /= 1800;             
  Serial.println(" calibration finished!");                                //1800 measures are done!
                                  
 digitalWrite(3,1);
  Serial.println("system is starting");
                   Serial.println(receiver_input_channel_3);
                Serial.println(receiver_input_channel_4);

while (( (receiver_input_channel_3 > 1050) || (receiver_input_channel_3 < 900)) ||  (receiver_input_channel_4 < 1200) ){       // activation of controller prior start controlling
                state =0;                          
                start++;
       stop_bipping ()  ;                                    
                if(start == 250){                              
                digitalWrite(2,b);                                           // led blinking to understand that we wait for starting
                b=!b;
                start = 0; 
                Serial.println("controller not connected or wrong throttle");
                         
                }
 }
        Serial.println("started control ");

  zero_timer = micros();                                   //Set the zero_timer for the first loop.

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  recordGyroRegisters();
  rotXn = gyroX *0.0305/2;                           // 2^15 with (MSB for sign) =  32768/ 500 deg (full scale) = 65.53 = 0.0305*2
  rotYn = gyroY *0.0305/2; 
  rotZn = gyroZ *0.0305/2;
  rotX = rotXo *0.8+0.2*rotXn;
  rotY = rotYo *0.8+0.2*rotYn;              // complementary filter to bypass wrong pick data
  rotZ = rotZo *0.8+0.2*rotZn;
  rotXo=rotX;
  rotYo=rotY;
  rotZo=rotZ;
  k= receiver_input_channel_5-900;
  k=k*0.004;
  //Serial.println (k);
 //   Serial.println (receiver_input_channel_5);
y_p_gain = k;
 // r_p_gain = k;
 // p_p_gain =k;
  
if (((receiver_input_channel_3 < 1050) && (receiver_input_channel_3 >980)) && ((receiver_input_channel_4 > 980) &&(receiver_input_channel_4 < 1100)) ){
  state=1;                          // transition to control; left hand joystick down-left
     digitalWrite(2,0);
     digitalWrite(3,0);      
     digitalWrite(13,1);      

     }

 if(state == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
  state=2;                 // control mode active: left hand joystick down center
  i_roll=0;
i_pitch=0;                                        // PID reseting
i_yaw=0;
last_r_d_error=0;
last_p_d_error=0;
last_y_d_error=0;
 rotX = 0;
  rotY = 0; 
  rotZ = 0;
 }
  if(state == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950){
    state = 0;               // stop
      digitalWrite(2,1);
      digitalWrite(3,1);
      digitalWrite(13,0);      

  }

     r_setpoint = 0;       // rotation on X
  if(receiver_input_channel_1 > 1510) r_setpoint = (receiver_input_channel_1 - 1510)/4.0;
  else if(receiver_input_channel_1 < 1490) r_setpoint = (receiver_input_channel_1 - 1490)/4.0;
  r_setpoint = r_setpoint * -1;     // correct the rotation

     p_setpoint = 0;        // rotation on Y
  if(receiver_input_channel_2 > 1510)p_setpoint = (receiver_input_channel_2 - 1510)/4.0;
  else if(receiver_input_channel_2 < 1490)p_setpoint = (receiver_input_channel_2 - 1490)/4.0;
 

     y_setpoint = 0;        // rotation on Z
  if(receiver_input_channel_3 > 1040){  // no yaw on ground   - need to test and find minimal throttle 
    if(receiver_input_channel_4 > 1510)y_setpoint = (receiver_input_channel_4 - 1510)/5.0;
    else if(receiver_input_channel_4 < 1490)y_setpoint = (receiver_input_channel_4 - 1490)/5.0;
     y_setpoint =y_setpoint * -1;          // correct the rotation
  }   

PID_calc();
//Serial.println("PID CALC ");

//print_PID;
if (state ==2){
       if (receiver_input_channel_3 > 1850) receiver_input_channel_3 = 1850;    //maximaze throttle for better contrllability
     // throttle = (receiver_input_channel_3 -1000)/2.7 +1000;
             throttle = receiver_input_channel_3 ;

       timer_mot_1 = throttle  + pid_p - pid_y ;
       timer_mot_2 = throttle  + pid_r + pid_y ;
       timer_mot_3 = throttle  - pid_p - pid_y;
       timer_mot_4 = throttle  - pid_r + pid_y;
// no battery compensation so far
       if (timer_mot_1 < 1020) timer_mot_1 = 1020;                                         
       if (timer_mot_2 < 1020) timer_mot_2 = 1020;
       if (timer_mot_3 < 1020) timer_mot_3 = 1020;
       if (timer_mot_4 < 1020) timer_mot_4 = 1020;

       if(timer_mot_1 > 1900)timer_mot_1 = 1900;
       if(timer_mot_2 > 1900)timer_mot_2 = 1900;
       if(timer_mot_3 > 1900)timer_mot_3 = 1900;
       if(timer_mot_4 > 1900)timer_mot_4 = 1900;

}
else{
    timer_mot_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    timer_mot_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    timer_mot_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    timer_mot_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  while(zero_timer + 4000 > micros());                     //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
  PORTD |= B11110000;

  timer_mot_1 +=  zero_timer;  
  timer_mot_2 +=  zero_timer;
  timer_mot_3 +=  zero_timer;
  timer_mot_4 +=  zero_timer;

     while(PORTD >= 16){  
    esc_loop_timer = micros();                               //Check the current time.
    if(timer_mot_1 <= esc_loop_timer)PORTD &= B11101111; //When the delay time is expired, digital port 8 is set low.
    if(timer_mot_2 <= esc_loop_timer)PORTD &= B11011111; //When the delay time is expired, digital port 9 is set low.
    if(timer_mot_3 <= esc_loop_timer)PORTD &= B10111111; //When the delay time is expired, digital port 10 is set low.
    if(timer_mot_4 <= esc_loop_timer)PORTD &= B01111111; //When the delay time is expired, digital port 11 is set low.
     }
 //printData();
// printData2();

//if (receiver_input_channel_5 > 1600)digitalWrite(1,0);          // mode Auto landing
//if (receiver_input_channel_5 < 1600)digitalWrite(1,0);           // No mode auto

 // tt=micros()- alpha;
 // Serial.println(tt);
}

void setup_gyro(){
  Wire.beginTransmission(105); //I2C address of the MPU
  Wire.write(0x20)         ; //Accessing the register CTRL_REG1 (20h) - Gyroscope Configuration (page 29/42) 
  Wire.write(0x0F);                   //Setting the gyro to full scale -- data rate at 100 hz  -- normal mode -- enable axes
  Wire.endTransmission();

  Wire.beginTransmission(105);                       //Start communication with the gyro (adress 1101001)
  Wire.write(0x23);                                  //We want to write to register CTRL_REG4 (23h)
  Wire.write(0x90);                                  //Set the register bits as 10010000 (Block Data Update active by reading-- 00: 500 deg ful scale -- self test disable -- 4 wires
  Wire.endTransmission();  
}

void recordGyroRegisters() {
  Wire.beginTransmission(105);                       //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                   //Start reading @ register 28h (40) + auto increment with every read by 1 at MSB and x0000000 for LSB (128) 
                                                     //  L3G4200D behaves like a slave device page 22/42-- The 7 LSB: actual reg address -- MSB enables address auto-increment 
  Wire.endTransmission();                            //End the transmission
  Wire.requestFrom(105, 6);                          //Request 6 bytes from the gyro
  while(Wire.available() < 6);
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyroX = ((highByte<<8)|lowByte);               //Multiply highByte by 256 and ad lowByte
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyroY = ((highByte<<8)|lowByte);              //Multiply highByte by 256 and ad lowByte
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyroZ = ((highByte<<8)|lowByte);                //Multiply highByte by 256 and ad lowByte
  //gyroZ=gyroZ* -1;  
  if(cal_int == 1800){         // execute just when finish calibration
  gyroX -= gyroXc;
  gyroY -= gyroYc;
  gyroZ -= gyroZc;  

}
}



ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  
    if(last_channel_1 == 0 && PINB & B00000001) {                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  
  else if((last_channel_1 == 1) && !(PINB & B00000001)){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }

 //Channel 2=========================================
      if(last_channel_2 == 0 && PINB & B00000010) {                                   //Input 8 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_1 to current_time
    }
  
  else if((last_channel_2 == 1) && !(PINB & B00000010)){                                //Input 8 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 1 is current_time - timer_1
  }

 //Channel 3=========================================
  
    if(last_channel_3 == 0 && PINB & B00000100) {                                   //Input 8 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_1 to current_time
    }
  
  else if((last_channel_3 == 1) && !(PINB & B00000100)){                                //Input 8 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 1 is current_time - timer_1
  }
 //Channel 4=========================================
  
    if(last_channel_4 == 0 && PINB & B00001000) {                                   //Input 8 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_1 to current_time
    }
  
  else if((last_channel_4 == 1) && !(PINB & B00001000)){                                //Input 8 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 1 is current_time - timer_1
  }
   //Channel 5=========================================
  
    if(last_channel_5 == 0 && PINB & B00010000) {                                   //Input 8 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_1 to current_time
    }
  
  else if((last_channel_5 == 1) && !(PINB & B00010000)){                                //Input 8 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_input_channel_5 = current_time - timer_5;         //Channel 1 is current_time - timer_1
  }
  }
void printData() {
  Serial.print("M-1-- ");
  Serial.print(timer_mot_1 - zero_timer);
  Serial.print("/--M-2-- ");
  Serial.print(timer_mot_2 - zero_timer);
  Serial.print("/--M-3-- ");
  Serial.print(timer_mot_3 - zero_timer);
  Serial.print("/--M-4-- ");
  Serial.print(timer_mot_4 - zero_timer);
  Serial.print("/-- CH-5-- ");
  Serial.print(receiver_input_channel_5);
  Serial.print("--State-- ");
  Serial.print(state);
  Serial.print("--pid_r-- ");
  Serial.print(pid_r);
  Serial.print("/--pid_p-- ");
  Serial.print(pid_p);
  Serial.print("/--pid_y-- ");
  Serial.print(pid_y);
   Serial.print("--Roll-- ");
  Serial.print(rotX);
  Serial.print("/--Pitch-- ");
  Serial.print(rotY);
  Serial.print("/--Yaw-- ");
  Serial.println(rotZ);
}

void printData2() {
  Serial.print("/--PID Y-- ");
  Serial.print(pid_y);
  Serial.print("/--set point Yaw-- ");
  Serial.print(y_setpoint);
  Serial.print("/-*-Yaw-*- ");
  Serial.println(rotZ);

  
}


void stop_bipping(){
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3); 
}


void ESC_calibration(){
    for (tt = 0; tt < 1000 ; tt ++){
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(2000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(2000);
    }
tt=0;
for (tt = 0; tt < 1000 ; tt ++){
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);
    }

}
void PID_calc(){
  correction ++;
////////////////////////////////////////////////////////////////////////////
r_error = r_setpoint - rotX;
//Serial.println("r_error");
//Serial.println(r_error);

i_roll += r_i_gain * r_error;
//Serial.println("i_roll");
//Serial.println(i_roll);
if (i_roll > r_max) i_roll = r_max;
else if (i_roll < r_max * -1) i_roll = r_max * -1;
pid_r = r_p_gain * r_error + i_roll + r_d_gain * (r_error - last_r_d_error);
last_r_d_error = r_error;

if (pid_r > r_max) pid_r = r_max;
else if (pid_r < r_max * -1 ) pid_r = r_max * -1;

////////////////////////////////////////////////////////////////////////////
p_error=p_setpoint-rotY;
i_pitch+=p_i_gain*p_error;
if (i_pitch > p_max)i_pitch=p_max;
else if (i_pitch <p_max*-1)i_pitch=p_max*-1;
pid_p= p_p_gain*p_error + i_pitch + p_d_gain*(p_error-last_p_d_error);
last_p_d_error=p_error;

if (pid_p > p_max) pid_p=p_max;
else if (pid_p <p_max* -1)pid_p=p_max* -1;

////////////////////////////////////////////////////////////////////////////

y_error=y_setpoint-3*rotZ;
i_yaw+=y_i_gain*y_error;
if (i_yaw > y_max)i_yaw=y_max;
else if (i_yaw <y_max*-1)i_yaw=y_max*-1;
pid_y= y_p_gain*y_error + i_yaw + y_d_gain*(y_error-last_y_d_error);
last_y_d_error=y_error;

if (pid_y > y_max) pid_y=y_max;
else if (pid_y <y_max* -1)pid_y=y_max* -1;
  
////////////////////////////////////////////////////////////////////////////
//if (correction == 2000){              // reset of Integration
//  i_roll=0;
//  i_pitch=0;
//  i_yaw=0;
//  correction=0;
//}

}





