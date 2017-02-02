// ------------------------------------------------ -------------------------------------------------- -------------------------------------------------- ---------------------
// ------------ Program for calculating the inclination angle from the gyro and acceleration data using a Kalman filter ------------------- ---------------
// -------------------------- and PID control for the serial control of the two motors by Sabertooth motor driver -------- ---------------------------------------
// ------------------------------------------------ -------------------------------------------------- -------------------------------------------------- ---------------------


#include "Wire.h"
#include "I2Cdev.h" // I2Cdev and MPU6050 must be installed as libraries
#include "MPU6050.h" // class default I2C address is 0x68 = AD0 low
#include <math.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial (NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST (SWSerial); // Use SWSerial as the serial port.

MPU6050 accelgyro;

Int16_t ax, ay, az; // Acceleration values ​​in the x, y and z directions of the MPU6050 sensor
Int16_t gx, gy, gz; // angular velocity values ​​in the x, y and z directions of the MPU6050 sensor

#define Pin_Lenkung_rechts 12 // Pin connector for the steering command Right
#define Pin_Lenkung_links 13 // Pin connector for the steering command Left

#define Pin_Schalter_Box 3 // Pin connector of the box switch to select between motor synchronization and I control

#define Pin_PID_control_P A1 // Pin connector for the potentiometer for changing the P-component
#define Pin_PID_control_I A2 // Pin connection for the potentiometer for changing the I-component
#define Pin_PID_regulation_D A3 // Pin connector for the potentiometer to change the D component


Int LoopTime_Soll = 9; // the desired grinding time in ms to reach the 100 Hz      
Int LoopTime_Adjust = LoopTime_Soll; // last loop time with forced pause
Int LoopTime_Bisher = LoopTime_Soll; // last loop time without forced pause
Unsigned long LoopTime_Start = 0; // Start time of the loop

Float angle; // Current angle of inclination
Float angle_set; // Setpoint of the inclination angle, ie 0 °
Float angle_border; // maximum permitted inclination angle, above which the Segway is switched off   

Float ACC_angle; // Angle from acceleration sensor
Float GYRO_rate; // Angle speed from the gyro sensor

Float Kp, Ki, Kd, ​​K; // Constants for PID control, differential, integral part, differential part, total part
Int motor; // value obtained by the PID control for motor control
Int Motor_rechts, Motor_links; // values ​​for the two motors
Float K_Motor_links, K_Motor_rechts; // Correction factors for synchronous operation of the two motors

Int Switch_Box; // variable, which scans the switch position on the box

Int steering_input_right = 0; // Variable for detecting a steering command to the right
Int steering_input_links = 0; // Variable for detecting a steering command to the left
Float steering_max; // Value by which the motor control should change as a maximum for a steering command
Float steering_right, steering_links; // Actual and stepwise increased control value when steering to the right or left


// ************************************************ ****************************
// ****************************** SET UP ***************** **********************
// ************************************************ ****************************


Void setup ()
   {
    Wire.begin (); 
     
    //SWSerial.begin(9600); // This is the baud rate you chose with the DIP switches.
     
    Serial.begin (9600); // baud rate for the serial monitor to check the values
    
    // initialize device
    Accelgyro.initialize ();
            
    CalibrateSensors (); // Subroutine for one-time calibration of the sensors
   }





// ************************************************ ***********************************
// ****************************** Calibration ***************** **********************
// ************************************************ ***********************************


Void calibrateSensors () // one-time determination of sensor zero values ​​(mean value from 50 measurements each)
   {
    
    // =============================================================================================== =
    // ======== Changing the sensor resolution ==========
    // =============================================================================================== =
    
    // =============================================================================================== ====================================================================================================
    // read raw accel / gyro measurements from device
    // Output value Acc: Resolution 2g: 16384 / g Resolution 4g: 8192 / g
    // Output Gyro: Resolution 250 ° / s: 131 / ° / s Resolution 500 ° / s: 65.5 / ° / s
    // =============================================================================================== ====================================================================================================
    
    Accelgyro.setFullScaleAccelRange (MPU6050_ACCEL_FS_2);
    Accelgyro.setFullScaleGyroRange (MPU6050_GYRO_FS_500);
       
     
    
    // Attempt to avoid the "howling" of the two motors at the beginning
    
    ST.motor (1, 0);
    ST.motor (2, 0);
    
   
        
    Angle_set = 0.0; // Setpoint for the angle of inclination
    Angle_border = 30.0; // maximum allowed inclination angle
    
    
    // ************************************************ *******
    // ********** K - Values ​​for the PID control *************
    // ************************************************ *******
    
    
    Kp = analogRead (pin_PID_regulation_P) * 25.0 / 1023.0; // Difference ratio with Poti
    Ki = 0.1; // integral part set with potentiometer (switch but possibly on motor correction, therefore set with 0.1 first)   
    Kd = analogRead (pin_PID_control_D) * 100.0 / 1023.0; // Differential ratio with potentiometer
    K = 1.0; // Total share


    // ************************************************ **
    // ********** K - Values ​​for the motors *************
    // ************************************************ **


    PinMode (Pin_Schalter_Box, INPUT); // Pin for selection between I control and motor synchronization
    
    K_Motor_rechts = 1.0; // Correction factor for the right motor
    K_Motor_links = 0.8; // Correction factor for the left motor
      
    
    // **********************************************
    // ********** Steering values ​​*************
    // **********************************************
    
    
    Steering_max = 25.0; // Value by which the motor control should change with a steering command MAXIMAL
    Steering_right = 0.0; // current additional value during the steering process to the right
    Steering_links = 0.0; // current additional value during the steering process to the left
 
    PinMode (Pin_Lenk_rechts, INPUT); // Pin for steering to the right is declared as input
    PinMode (Pin_Lenk_links, INPUT); // Pin for steering to the left is declared as input
      
   }






// ************************************************ ************************************************** *************************************************
// ************************************************ ************************************************** *************************************************
// ************************************************ ********************** HEADLESS *************************** ***********************************
// ************************************************ ************************************************** *************************************************
// ************************************************ ************************************************** *************************************************

Void loop ()
   {

   // ************************************************ *******
   // ********************* Sensor interrogation *******************
   // ************************************************ *******
   
    // =============================================================================================== ====================================================================================================
    // read raw accel / gyro measurements from device
    // Output value Acc: Resolution 2g: 16384 / g Resolution 4g: 8192 / g
    // Output Gyro: Resolution 250 ° / s: 131 / ° / s Resolution 500 ° / s: 65.5 / ° / s
    // =============================================================================================== ====================================================================================================
    
    Accelgyro.getMotion6 (& ax, & ay, & az, & gx, & gy, & gz);
   
    ACC_angle = atan (ay * 1.0 / az * 1.0) * 180.0 / 3.141592654; // Resolution 2g: 16384 / g
   
    // ACC_angle = atan ((ay / 16384.0) / (az / 16384.0)) * 180.0 / 3.141592654; // Resolution 2g: 16384 / g
    
    GYRO_rate = gx / 65.5; // Resolution 500 ° / s: 65.5 / ° / s

     
     
   // ************************************************ *******
   // ********** K - Values ​​for the PID control *************
   // ************************************************ *******
    
    
    Kp = analogRead (pin_PID_regulation_P) * 25.0 / 1023.0; // Define difference with Poti; Maximum = 25
    Kd = analogRead (pin_PID_control_D) * 100.0 / 1023.0; // differential ratio with potentiometer; Maximum = 100

    Switch_Box = digitalRead (Pin_Schalter_Box); // Query the pin for the switch state on the box
    
    If (Switch_Box == HIGH) // Activated by the switch on the I control box
       {
        Ki = analogRead (pin_PID_control_I) * 2.0 / 1023.0; // integral part with potentiometer; Maximum = 2        
       }
    Else // Switch on the motor control box by means of a switch
        {
         K_Motor_rechts = analogRead (pin_PID_control_I) * 2.0 / 1023.0; // correction factor for synchronous operation of the two motors; Maximum = 2
        }

      
     
     
     // ************************************************ ********************************
     // ****************** Kalman filter, PWM calculation and motor values ​​*****************
     // ************************************************ ********************************
     
     
     Angle = kalmanCalculate (ACC_angle, GYRO_rate, LoopTime_Adjust); // Calculated angle with Kalman filter

     
     If (angle> angle_limit || angle <(angle_limit * (-1)))
        {
         // =================================================================================================
         // Abort due to too great an angle of inclination!
         // =================================================================================================
         
         ST.motor (1, 0);
         ST.motor (2, 0);
        }
     Else
        {
         // =================================================================================
         // Angle of inclination in order
         // =================================================================================
      
 
         Motor = pid (angle, angle_set, GYRO_rate); // Calculation of the PWM value for controlling the motors
     
         Motor_rechts = K_Motor_rechts * Motor; // Calculation of the motor speed, which is synchronized with K-factor, for the right motor

         Motor_links = K_Motor_links * Motor; // Calculation of the engine speed, synchronized with K-factor, for the left engine
     
         
          
         // ************************************************ **************************************
         // ***** Check whether the steering has been activated and change the motor control *****
         // ************************************************ **************************************
     
     
         Lenkung_Eingang_rechts = digitalRead (Pin_Lenkung_rechts); // Query the pin for steering to the right

         If (steering_input_right == HIGH)
            {     
              // ******************************************
              // *** Steering to the right has been pressed ***
              // ******************************************
          
              If (Motor_rechts> = 0) // segway moves forward or stands. Which motor is interrogated does not matter.
                 {
                  Motor_rechts = Motor_rechts - (int) steering_right; // Perhaps also try to multiply a factor (eg * (1 - 0.1))
                  Motor_links = Motor_links + (int) Steering_right; // Perhaps also try to multiply a factor (eg * (1 + 0.1))
                 }
              Else // segway just goes backwards
                 {
                  Motor_rechts = Motor_rechts + (int) steering_right; // Perhaps also try to multiply a factor (eg * (1 + 0.1)
                  Motor_links = Motor_links - (int) steering_right; // Perhaps also try to multiply a factor (eg * (1 - 0.1))
                 }
                 
             Steering_right = steering_right + 0.05; // Better only to increase eg 0.1 per query so that steering is not too abrupt!
             
             If (steering_right> steering_max) steering_right = steering_max; // Do not exceed the maximum steering value!
             
             // steering_right = constrain (steering_right, 0, steering_max); // right steering value in the interval [0, steering_max]
            } 
         Else
            {
             Steering_right = 0.0;
            }
    
    
         Lenkung_Eingang_links = digitalRead (Pin_Lenk_links); // Query the pin for steering to the left

         If (steering_input_links == HIGH)
            {     
              // *****************************************
              // *** steering to the left has been pressed ***
              // *****************************************
          
              If (Motor_links> = 0) // segway moves forward or stands. Which engine is queried no matter.
                 {
                  Motor_rechts = Motor_rechts + (int) steering_links; // Perhaps also try to multiply a factor (eg * (1 + 0.1)
                  Motor_links = Motor_links - (int) steering_links; // Perhaps also try to multiply a factor (eg * (1 - 0.1))
                 }
              Else // segway just goes backwards
                 {
                  Motor_rechts = Motor_rechts - (int) steering_links; // Perhaps also try to multiply a factor (eg * (1 - 0.1))
                  Motor_links = Motor_links + (int) Steering_links; // Perhaps also try to multiply a factor (eg * (1 + 0.1)
                 }
                 
             Steering_links = steering_links + 0.05; // Better only to increase eg 0.1 per query so that steering is not too abrupt!
             
             If (steering_links> steering_max) steering_links = steering_max; // Do not exceed the maximum steering value!
             
             // steering_links = constrain (steering_links, 0, steering_max); // left steering value in the interval [0, steering_max]
            } 
         Else
            {
             Steering_links = 0.0;
            }
       
        
        
     
         // ************************************************ *******************************************
         // ******************************** Controlling the motors ************* **********************
         // ************************************************ *******************************************
        
         
         Motor_rechts = constrain (Motor_rechts, -127, 127); // right motor value in the interval [-127,127]
         Motor_links = constrain (Motor_links, -127, 127); // left motor value is brought into the interval [-127,127]
         
                      
     / *
         // Use of a root function instead of the linear control function to improve the response at low motor values
         // =================================================================================================== ==================================================================================================== ==================================================================================================
         
         If (Motor_right> = 0) // Right motor turns forward
            { 
             Motor_rechts = sqrt (127 * Motor_right); // to improve the response at low motor values
              
             ST.motor (2, motor_right);      
            }
         Else // right motor turns backward
            {
             Motor_rechts = -sqrt (127 * motor_right); // to improve the response at low motor values
             
             ST.motor (2, motor_right);               
            }


         If (engine_links> = 0) // left engine turns forward
            {
             Motor_links = sqrt (127 * engine_links); // to improve the response at low motor values
             
             ST.motor (1, engine_links);               
            }
         Else // left engine turns backwards
            {
             Motor_links = -sqrt (127 * motor_links); // to improve the response at low motor values
             
             ST.motor (1, motor_links);  
            }
         * /
         
         ST.motor (1, motor_links);
         ST.motor (2, motor_right);
         
        } 


   // ************************************************ ************************ 
   // *********************** Output of measured values ​​********************** ****
   // ************************************************ ************************

    Value output ();


   // ************************************************ ******************
   // *********************** Keyboard Query ************************ **
   // ************************************************ ******************

   // key input ();



   // ************************************************ **********************
   // *********************** loop timing control ********************** ****
   // ************************************************ **********************

     LoopTime_Bisher = millis () - LoopTime_Start; // Time since the last loop
     
     If (LoopTime_Bisher <LoopTime_Soll)
        {
         Delay (LoopTime_Soll - LoopTime_Bisher); // Delay to get the same loop time
        }
     
     LoopTime_Adjust = millis () - LoopTime_Start; // updated duration of the last loop, should equal LoopTime_Soll = eg 10 msek!
     LoopTime_Start = millis (); // new start of the loop
   
 }


// ************************************************ ********************************************
// ****************** Value output to the serial interface ************************* *****
// ************************************************ ********************************************

Void Value output ()
   {
    / *
    Serial.print (angle);
    Serial.print ("");
    Serial.println (motor);
    Serial.print ("");
    * /
    
    Serial.print ("a_y =");
    Serial.print (ay / 16384.0);
    Serial.print ("a_z =");
    Serial.print (az / 16384.0);
    Serial.print ("ACC_angle =");
    Serial.print (ACC_angle, 0);
    Serial.print ("GYRO_rate =");
    Serial.print (GYRO_rate, 0);
    Serial.print ("angle:");
    Serial.println (angle, 0);
    
    / *
    Serial.print ("Motor:");
    Serial.print (motor);
    Serial.print ("Motor_rechts:");
    Serial.print (Motor_rechts);
    Serial.print ("Motor_links:");
    Serial.println (Motor_links);
    * /
    
   }


// ************************************************ ************************************************** ****
// ***************************************** PID CONTROL **** ******************************************
// ************************************************ ************************************************** ****

Float error;
Float last_error = 0;
Float pTerm;
Float iTerm;
Float dTerm;
Float integrated_error = 0;
Int GUARD_GAIN = 40; // maximum integrated angular error

   Int pid (float angle_actual, float angle_delivery, float angular speed)
      {
       Error = Winkel_Vorgabe - Winkel_aktuell;
       
       PTerm = Kp * error; // Difference share
       
       
       Integrated_error = integrated_error + error;
   
       ITerm = Ki * constrain (integrated_error, -GUARD_GAIN, GUARD_GAIN); // Integral component
  
       
       DTerm = Kd * angular speed / 100.0; // differential ratio; : 100 to reach usable values!
       
       / *
       Serial.print ("K_p:");
       Serial.print (pTerm);
       Serial.print ("K_d:");
       Serial.println (dTerm);
       * /
  
       Last_error = error;
  
       // Serial.println (K * (pTerm + iTerm + dTerm));
       
  
       Return constrain (K * (pTerm + iTerm + dTerm), -127,127); // Output of the motor value for the two motors within the limits [-127,127]
      } 

 
 

// ************************************************ ************************************************** ****
// ************************************** Kalman filter module ******* ***********************************
// ************************************************ ************************************************** ****


    Float Q_angle = 0.001; // E (alpha2) = 0.001
    Float Q_gyro = 0.003; // E (bias2) = 0.003
    Float R_angle = 0.001; // Sz = 0.03 !!! The larger the number, the more insensitive the angle reacts to changes !!!
    Float x_angle = 0;
    Float x_bias = 0;
    Float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
    Float dt, y, S;
    Float K_0, K_1;

  Float kalmanCalculate (float newAngle, float newRate, int looptime)
     {
      Dt = float (looptime) / 1000; // dt in seconds
      X_angle = x_angle + dt * (newRate - x_bias);
      P_00 = P_00 - dt * (P_10 + P_01) + Q_angle * dt;
      P_01 = P_01 - dt * P_11;
      P_10 = P_10 - dt * P_11;
      P_11 = P_11 + Q_gyro * dt;

      Y = newAngle - x_angle;
      S = P_00 + R_angle;
      K_0 = P_00 / S;
      K_1 = P_10 / S;

      X_angle + = K_0 * y;
      X_bias + = K_1 * y;
      P_00 - = K_0 * P_00;
      P_01 - = K_0 * P_01;
      P_10 - = K_1 * P_00;
      P_11 - = K_1 * P_01;

      Return x_angle;
     }



// ************************************************ ********************
// ******** Keyboard query to change the PUI parameters *********
// ************************************************ ********************

Int Keyboard input ()
   {
    If (! Serial.available ()) return 0;
   
    Char param = Serial.read (); // get parameter byte
  
    If (! Serial.available ()) return 0;
  
    Char cmd = Serial.read (); // get command byte
  
    Serial.flush ();
  
    Switch (param)
       {
        Case 'p':
           If (cmd == '+') Kp ++;
           If (cmd == '-') Kp--;
           Break;
        Case 'i':
           If (cmd == '+') Ki + = 0.1;
           If (cmd == '-') K i = 0.1;
           Break;
        Case 'd':
           If (cmd == '+') Kd ++;
           If (cmd == '-') Kd--;
           Break;
       Case 'k':
           If (cmd == '+') K + = 0.2;
           If (cmd == '-') K = 0.2;
           Break;
       Case 'l':
           If (cmd == '+') K_Motor_links + = 0.1;
           If (cmd == '-') K_Motor_links - = 0.1;
           Break;
       Case 'r':
           If (cmd == '+') K_Motor_rechts + = 0.1;
           If (cmd == '-') K_Motor_rechts - = 0.1;
           Break;
     
       default:
           Serial.print ("?"); Serial.print (param);
           Serial.print ("?"); Serial.println (cmd);
      }
  
    Serial.println ();
    Serial.print ("K:"); Serial.print (K);
    Serial.print ("Kp:"); Serial.print (Kp);
    Serial.print ("Ki:"); Serial.print (Ki);
    Serial.print ("Kd:"); Serial.print (Kd);
    Serial.print ("K_Motor_links:"); Serial.print (K_Motor_links);
    Serial.print ("K_Motor_rechts:"); Serial.println (K_Motor_rechts);
   } 


