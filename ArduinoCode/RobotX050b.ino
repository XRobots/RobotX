#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define OFFSET1  2060;  //  2040 right ankle - lower value tips inwards
#define OFFSET2  2000;  //  2030 right lower leg - lower value is more bent
#define OFFSET3  2285;  //  2315 right upper leg - lower value is more bent
#define OFFSET4  2280;  //  2050 (*replaced pot*) right hip - higher moves in
#define OFFSET5  2480;  //  2450 left ankle - higher value tips outwards
#define OFFSET6  2180;  //  left lower leg
#define OFFSET7  2380;  //  left upper leg
#define OFFSET8  2145;  //  left hip - higher moves in

#define OFFSET9  2150;   // front back body pot  - higher number back it move back

int deadspot = 0;
int deadspot2 = 0;

int remote;
int count = 0;

int but1;
int but2;
int but3;
int but4;
int but5;
int but6;
int but7;

int pot1;
int pot2;
int pot3;
int pot4;
int pot5;
int pot6;
int pot7;
int pot8;

int pot9; // front back body action

int volume; // motor max speed knob

int state01 = 0;
int stateLeft = 0;
int stateRight = 0;
int straightLeft = 0;
int straightRight = 0;

int motor1 = 0;
int motor1a = 0;
int motor1b = 0;

int motor2 = 0;
int motor2a = 0;
int motor2b = 0;

int motor3 = 0;
int motor3a = 0;
int motor3b = 0;

int motor4 = 0;
int motor4a = 0;
int motor4b = 0;

int motor5 = 0;
int motor5a = 0;
int motor5b = 0;

int motor6 = 0;
int motor6a = 0;
int motor6b = 0;

int motor7 = 0;
int motor7a = 0;
int motor7b = 0;

int motor8 = 0;
int motor8a = 0;
int motor8b = 0;

double IMU1;
double IMU2;
double IMU3a;
double IMU3b;

unsigned long previousMillis = 0;
const long interval = 10;

unsigned long previousStepMillis = 0;

double IMUtrack01;
double IMUtrack01_av;

const int numReadings = 500;

double readings[numReadings];      // the readings from the analog input    // IMUtrack01 running average
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average


double Pk1 = 5;   // right ankle
double Ik1 = 0;
double Dk1 = 0.2;

double Pk2 = 25; 
double Ik2 = 0;
double Dk2 = 0.2;

double Pk3 = 25; 
double Ik3 = 0;
double Dk3 = 0.2;

double Pk4 = 10;   // right hip
double Ik4 = 0;
double Dk4 = 0.2;

double Pk5 = 5;  // left ankle
double Ik5 = 0;
double Dk5 = 0.2;

double Pk6 = 25; 
double Ik6 = 0;
double Dk6 = 0.2;

double Pk7 = 25; 
double Ik7 = 0;
double Dk7 = 0.2;

double Pk8 = 10;  // left hip
double Ik8 = 0;
double Dk8 = 0.2;

double Pka = 20;   // IMU 1 - left leg hip
double Ika = 0;
double Dka = 0.1;

double Pkb = 20;   // IMU 2 - right leg hip
double Ikb = 0;
double Dkb = 0.1;

double Pka2 = 20;   // IMU 1 - left leg ankle
double Ika2 = 0;
double Dka2 = 0.1;

double Pkb2 = 20;   // IMU 2 - right leg ankle
double Ikb2 = 0;
double Dkb2 = 0.1;

double Pkc = 40;   // IMU 3 - body side to side
double Ikc = 0;
double Dkc = 0.5;

double Pk100 = 2;      // Back/front servo
double Ik100 = 1;
double Dk100 = 0.1;

double Pk101 = 50;      // Back/front stability
double Ik101 = 0;
double Dk101 = 0.5;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

double Setpoint2, Input2, Output2, Output2a;
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

double Setpoint3, Input3, Output3, Output3a;
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);

double Setpoint4, Input4, Output4, Output4a;
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);

double Setpoint5, Input5, Output5, Output5a;
PID PID5(&Input5, &Output5, &Setpoint5, Pk5, Ik5 , Dk5, DIRECT);

double Setpoint6, Input6, Output6, Output6a;
PID PID6(&Input6, &Output6, &Setpoint6, Pk6, Ik6 , Dk6, DIRECT);

double Setpoint7, Input7, Output7, Output7a;
PID PID7(&Input7, &Output7, &Setpoint7, Pk7, Ik7 , Dk7, DIRECT);

double Setpoint8, Input8, Output8, Output8a;
PID PID8(&Input8, &Output8, &Setpoint8, Pk8, Ik8 , Dk8, DIRECT);

double Setpointa, Inputa, Outputa;
PID PIDa(&Inputa, &Outputa, &Setpointa, Pka, Ika , Dka, DIRECT);  // IMU1 - left leg hip

double Setpointb, Inputb, Outputb;
PID PIDb(&Inputb, &Outputb, &Setpointb, Pkb, Ikb , Dkb, DIRECT);  // IMU2 - right leg hip

double Setpointa2, Inputa2, Outputa2;
PID PIDa2(&Inputa2, &Outputa2, &Setpointa2, Pka2, Ika2 , Dka2, DIRECT);  // IMU1 - left leg ankle

double Setpointb2, Inputb2, Outputb2;
PID PIDb2(&Inputb2, &Outputb2, &Setpointb2, Pkb2, Ikb2 , Dkb2, DIRECT);  // IMU2 - right leg ankle

double Setpointc, Inputc, Outputc;
PID PIDc(&Inputc, &Outputc, &Setpointc, Pkc, Ikc , Dkc, DIRECT);  // IMU3 - body side to side

double Setpoint100, Input100, Output100, Output100a;
PID PID100(&Input100, &Output100, &Setpoint100, Pk100, Ik100 , Dk100, DIRECT);  // back/front body servo

double Setpoint101, Input101, Output101, Output101a;
PID PID101(&Input101, &Output101, &Setpoint101, Pk101, Ik101 , Dk101, DIRECT);  // back/front body stability

double fbStab;
double ssStab;



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  delay(500);
  pwm.begin();
  delay(500);
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  delay(500);

  pinMode(23, INPUT);
  pinMode(25, INPUT);
  pinMode(27, INPUT);
  pinMode(29, INPUT);
  pinMode(31, INPUT);
  pinMode(33, INPUT);
  pinMode(35, INPUT);

  pinMode(4,OUTPUT);   // PWM - back front body servo
  pinMode(5,OUTPUT);

  analogReadResolution(12);

  PID1.SetMode(AUTOMATIC);              // PID Setup - right ankle
  PID1.SetOutputLimits(-1000, 1000);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              // PID Setup - right lower leg
  PID2.SetOutputLimits(-1500, 1500);
  PID2.SetSampleTime(10);

  PID3.SetMode(AUTOMATIC);              // PID Setup - right upper leg
  PID3.SetOutputLimits(-1500, 1500);
  PID3.SetSampleTime(10);

  PID4.SetMode(AUTOMATIC);              // PID Setup - right hip
  PID4.SetOutputLimits(-800, 800);
  PID4.SetSampleTime(10);

  PID5.SetMode(AUTOMATIC);              // PID Setup - left ankle
  PID5.SetOutputLimits(-1000, 1000);
  PID5.SetSampleTime(10);

  PID6.SetMode(AUTOMATIC);              // PID Setup - left lower leg
  PID6.SetOutputLimits(-1500, 1500);
  PID6.SetSampleTime(10);

  PID7.SetMode(AUTOMATIC);              // PID Setup - left upper leg
  PID7.SetOutputLimits(-1500, 1500);
  PID7.SetSampleTime(10);

  PID8.SetMode(AUTOMATIC);              // PID Setup - left hip
  PID8.SetOutputLimits( -800, 800);
  PID8.SetSampleTime(10);

  PIDa.SetMode(AUTOMATIC);              // PID Setup - IMU 1 - left leg hip
  PIDa.SetOutputLimits( -150, 150);
  PIDa.SetSampleTime(10);

  PIDb.SetMode(AUTOMATIC);              // PID Setup - IMU 2 - right leg hip
  PIDb.SetOutputLimits( -150, 150);
  PIDb.SetSampleTime(10);

  PIDa2.SetMode(AUTOMATIC);              // PID Setup - IMU 1 - left leg ankle
  PIDa2.SetOutputLimits( -150, 150);
  PIDa2.SetSampleTime(10);

  PIDb2.SetMode(AUTOMATIC);              // PID Setup - IMU 2 - right leg ankle
  PIDb2.SetOutputLimits( -150, 150);
  PIDb2.SetSampleTime(10);

  PIDc.SetMode(AUTOMATIC);              // PID Setup - IMU 3 - body
  PIDc.SetOutputLimits( -2048, 2048);
  PIDc.SetSampleTime(10);

  PID100.SetMode(AUTOMATIC);              // PID Setup - Back/front body servo
  PID100.SetOutputLimits( -512, 512);
  PID100.SetSampleTime(10);

  PID101.SetMode(AUTOMATIC);              // PID Setup - Back/front body stability
  PID101.SetOutputLimits( -512, 512);
  PID101.SetSampleTime(10);

      // **************** manual motor speed adjustment here **********************
    volume = analogRead(A0);   // ranges from 0 to 4096
    volume = (volume / 10) -200;      // ranges from -200 to +200


    PID1.SetOutputLimits((-1500-volume), (1500+volume));  // right ankle
    PID2.SetOutputLimits((-2500-volume), (2500+volume));  // right lower leg
    PID3.SetOutputLimits((-2500-volume), (2500+volume));  // right upper leg
    PID4.SetOutputLimits((-1024-volume), (1024+volume));  // right hip
    PID5.SetOutputLimits((-1500-volume), (1500+volume));  // left ankle
    PID6.SetOutputLimits((-2500-volume), (2500+volume));  // left lower leg
    PID7.SetOutputLimits((-2500-volume), (2500+volume));  // left upper leg
    PID8.SetOutputLimits((-1024-volume), (1024+volume));   // left hip  

    //PID100.SetOutputLimits((-255-volume), (255+volume));  // front / back body servo

  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(5, 0, 0);
  pwm.setPWM(6, 0, 0);
  pwm.setPWM(7, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(14, 0, 0);
  pwm.setPWM(15, 0, 0);
  delay(500);

    for (int thisReading = 0; thisReading < numReadings; thisReading++) {    // setup running average for IMUtrack01
    readings[thisReading] = 0;
  }


}

void loop() {

 unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {  //start timed event
    previousMillis = currentMillis;


    but1 = digitalRead(23);
    but2 = digitalRead(25);
    but3 = digitalRead(27);
    but4 = digitalRead(29);
    but5 = digitalRead(31);
    but6 = digitalRead(33);
    but7 = digitalRead(35);    

    if (Serial1.available() > 0) {  // IMU 1 - left leg
            // read the incoming byte:
            IMU1 = Serial1.parseFloat();
            IMU1 = IMU1 -0.8;                 
            if (Serial1.read() == '\n') {        
            }
    }

    if (Serial2.available() > 0) {  // IMU 2 - right leg
            // read the incoming byte:
            IMU2 = Serial2.parseFloat();
            IMU2 = IMU2 +2.5;                 
            if (Serial2.read() == '\n') {        
            }
    }

    if (Serial3.available() > 0) {  // IMU 2 - right leg
            // read the incoming byte:
            IMU3a = Serial3.parseFloat();
            IMU3b = Serial3.parseFloat();
            IMU3a = IMU3a + 0.7;        //front to back
            IMU3b = IMU3b +1.8;         // 1.8 Side to side                 
            if (Serial3.read() == '\n') {        
            }
    }



    Inputc = IMU3b;
    Setpointc = 0;

    PIDc.Compute();

   //Serial.print(IMU1);
   //Serial.print(" , ");
   //Serial.println(IMU3b);
   //Serial.print(" , ");
   //*****IMU tracking

   IMUtrack01 = (IMU1 + IMU2);

   // *** IMUtrack01  ***

    total = total - readings[readIndex];
    readings[readIndex] = IMU3b;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;

      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }

    IMUtrack01_av = total / numReadings;


    Serial.println(IMUtrack01_av);
     
     
   //Serial.println(IMUtrack01);

    // ********** front/back stability*************

    pot9 = analogRead(9);
    pot9 = pot9 - OFFSET9;



    Input101 = IMU3a;           // IMU PID for front/back stability
    Setpoint101 = -1;
    PID101.Compute();



    Input100 = pot9;            // Servo PID front/back
    Setpoint100 = constrain((Output101*-1),-150,150);
    PID100.Compute();


    if (Output100 > 0) {
      analogWrite(4,Output100);
      analogWrite(5,0);
    }
    else if (Output100 < 0) {
      Output100a = abs(Output100);
      analogWrite(5,Output100a);
      analogWrite(4,0);
    }


    fbStab = Output101/2;   // *************** front to back stability stuff


        if (but2 == 1) {                      // second toggle switch - state machine1

        if (state01 == 0 && stateRight == 0 && IMU3b <= 1 && currentMillis - previousStepMillis >= 500  ) { 
            previousStepMillis = currentMillis;

            stateLeft = 1;
            state01 = 1;
              }          

        if (state01 == 1 && stateLeft == 0 && IMU3b >= -1  && currentMillis - previousStepMillis >= 500  ) { 
            previousStepMillis = currentMillis;

                stateRight = 1;                
                state01 = 0;
              }

        }    // end of state machine1
    
                                   

    //******* Leg lifting ************

    if (stateLeft == 0) {  
                  if (but2 == 1){                                           // if the state machine is on
                      Setpoint2 = 0;// - fbStab;;
                      Setpoint3 = 0 + fbStab;
                      if (but7 == 1 && straightLeft == 1) {                  // push leg back if the button is pressed
                        Setpoint3 = 50 + fbStab;
                        Setpoint2 = -50;
                      }
                  }
                  else if(but2 == 0){                                       // if the state machine is off
                      Setpoint2 = 0 - fbStab;
                      Setpoint3 = 0 + fbStab;
                  }
    }
        else if (stateLeft == 1) {                                          // leg is triggered from state machine
                  Setpoint2 = -230;
                  Setpoint3 = -230;
                  stateLeft = 2;
                  straightLeft = 0;
            }

              if (stateLeft == 2 && Input2 < -200 && Input3 < -200) {  // if leg is bent
                  if (but7 == 1) {
                    Setpoint2 = -180;                                      // kick leg forward
                  }
                  stateLeft = 3;
              }

              if (stateLeft == 3 && IMU3b > -0) {
                stateLeft = 0;
              }                            
           

              if (Input2 > -60 && Input3 > -60 && straightLeft == 0) {      // if leg is straight
                straightLeft = 1;                                            // set straight flag
              }




    if (stateRight == 0) {
                if (but2 == 1 ) {                                       // if the state machine is on
                    Setpoint6 = 0;// - fbStab;
                    Setpoint7 = 0 + fbStab;
                    if (but7 == 1 && straightRight == 1) {              // push leg back if the button is pressed
                      Setpoint7 = 50 + fbStab;
                      Setpoint6 = -50;
                    }
                }                                                       // if the state machine is off
                else if (but2 == 0) {
                    Setpoint6 = 0 - fbStab;
                    Setpoint7 = 0 + fbStab;
                }
      }
          
          else if (stateRight == 1) {                                     // leg is triggered from state machine
                Setpoint6 = -230;
                Setpoint7 = -230;
                stateRight = 2;
                straightRight =0;
                }
                
                if (stateRight == 2 && Input6 < -200 && Input7 < -200) {  // if leg is bent
                  if (but7 == 1){
                    Setpoint6 = -180;                                          // kick leg forward               
                  }
                  stateRight = 3;
                }

                if (stateRight == 3 && IMU3b < 0) {
                  stateRight = 0;
                }
 
                if (Input6 > -60 && Input7 > -60 && straightRight == 0) {   // if leg is straight
                  straightRight = 1;                                        // set straight flag
                } 



    //******* Leg lifting End ************
    

    Inputa = IMU1;
    Inputb = IMU2;
    Inputa2 = IMU1;
    Inputb2 = IMU2;

    PIDa.Compute();
    PIDb.Compute();    
    PIDa2.Compute();
    PIDb2.Compute();

    pot1 = analogRead(A1) - OFFSET1;
    pot2 = analogRead(A2) - OFFSET2;
    pot3 = analogRead(A3) - OFFSET3;
    pot4 = analogRead(A4) - OFFSET4;
    pot5 = analogRead(A5) - OFFSET5;
    pot6 = analogRead(A6) - OFFSET6;
    pot7 = analogRead(A7) - OFFSET7;
    pot8 = analogRead(A8) - OFFSET8;

      

    if (but1 == 0) {                 //   first toggle switch - manual mode 

        if (but7 == 1) {
          Setpoint2 = -200;    // lower leg right
          Setpoint6 = -200;    // lower leg left
          Setpoint3 = -200;    // upper leg right
          Setpoint7 = -200;    // upper leg left
          
        }
        else if (but7 == 0) {
          Setpoint2 = 0; - fbStab + ssStab;  // lower leg
          Setpoint6 = 0; - fbStab - ssStab;  // lower leg
          Setpoint3 = 0; + fbStab + ssStab;
          Setpoint7 = 0; + fbStab - ssStab;
          
        }
    
        if (but6 == 1) {
          Setpoint1 = -150;
          Setpoint5 = 150;
          Setpoint4 = 150;
          Setpoint8 = -150;      
        }
    
        else if (but5 == 1) {
          Setpoint1 = 150;
          Setpoint5 = -150;
          Setpoint4 = -150;
          Setpoint8 = 150;      
        }
    
        else if (but6 == 0 && but5 == 0) {
          Setpoint1 = 0;
          Setpoint5 = 0;
          Setpoint8 = 0;
          Setpoint4 = 0;
        }

    }

      else if (but1 == 1) {           // side to side stability 

            if (but3 == 0 && but4 == 0) {
              Outputc = Outputc;              
            }

            else if (but4 == 1) {
              Outputc = Outputc + 150;

            }

            else if (but3 == 1){
               Outputc = Outputc - 150;
                     
            }

            
            Setpoint1 = 0 + constrain((Outputc/1.5),-180,180); // ankle
            Setpoint5 = 0 - constrain((Outputc/1.5),-180,180);  // ankle
            Setpoint8 = 0 + constrain(Outputc,-180,180);
            Setpoint4 = 0 - constrain(Outputc,-180,180);
            
            
                                
      }                               // end of dynamnic mode

    


    // *******************  PID and motor driving below here *******************
    
    // right ankle

    Input1 = pot1;
    //Setpoint1 = 0;
    PID1.Compute();

    
    Input2 = pot2;
    //Setpoint2 = 0;
    PID2.Compute();

    motor1 = (Output2 - Output1);
    motor2 = (Output2 + Output1);    

    if (motor1 < deadspot2)                                      
    {
    motor1a = abs(motor1);
    pwm.setPWM(0, 0, motor1a);
    pwm.setPWM(1, 0, 0);
    }
    else if (motor1 > deadspot)                         
    { 
    motor1b = abs(motor1);
    pwm.setPWM(0, 0, 0);
    pwm.setPWM(1, 0, motor1b);
    } 
    else
    {
    pwm.setPWM(0, 0, 0);  
    pwm.setPWM(1, 0, 0);
    }


    if (motor2 < deadspot2)                                      
    {
    motor2a = abs(motor2);
    pwm.setPWM(2, 0, motor2a);
    pwm.setPWM(3, 0, 0);
    }
    else if (motor2 > deadspot)                         
    { 
    motor2b = abs(motor2);
    pwm.setPWM(2, 0, 0);
    pwm.setPWM(3, 0, motor2b);
    } 
    else
    {
    pwm.setPWM(2, 0, 0);  
    pwm.setPWM(3, 0, 0);
    }

    // right hip

    Input3 = pot3;
    //Setpoint3 = 0;
    PID3.Compute();

    Input4 = pot4;
    //Setpoint4 = 0;
    PID4.Compute();


    motor3 = (Output3 - Output4);
    motor4 = (Output3 + Output4); 

    if (motor3 < deadspot2)                                      
    {
    motor3a = abs(motor3);
    pwm.setPWM(4, 0, motor3a);
    pwm.setPWM(5, 0, 0);
    }
    else if (motor3 > deadspot)                         
    { 
    motor3b = abs(motor3);
    pwm.setPWM(4, 0, 0);
    pwm.setPWM(5, 0, motor3b);
    } 
    else
    {
    pwm.setPWM(4, 0, 0);  
    pwm.setPWM(5, 0, 0);
    }


    if (motor4 < deadspot2)                                      
    {
    motor4a = abs(motor4);
    pwm.setPWM(6, 0, motor4a);
    pwm.setPWM(7, 0, 0);
    }
    else if (motor4 > deadspot)                         
    { 
    motor4b = abs(motor4);
    pwm.setPWM(6, 0, 0);
    pwm.setPWM(7, 0, motor4b);
    } 
    else
    {
    pwm.setPWM(6, 0, 0);  
    pwm.setPWM(7, 0, 0);
    }


    //rght foot

    Input5 = pot5;
    //Setpoint5 = 0;
    PID5.Compute();

    Input6 = pot6;
    //Setpoint6 = 0;
    PID6.Compute();

    motor5 = (Output6 + Output5);
    motor6 = (Output6 - Output5); 

    if (motor5 < deadspot2)                                      
    {
    motor5a = abs(motor5);
    pwm.setPWM(8, 0, motor5a);
    pwm.setPWM(9, 0, 0);
    }
    else if (motor5 > deadspot)                         
    { 
    motor5b = abs(motor5);
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, motor5b);
    } 
    else
    {
    pwm.setPWM(8, 0, 0);  
    pwm.setPWM(9, 0, 0);
    }


    if (motor6 < deadspot2)                                      
    {
    motor6a = abs(motor6);
    pwm.setPWM(10, 0, motor6a);
    pwm.setPWM(11, 0, 0);
    }
    else if (motor6 > deadspot)                         
    { 
    motor6b = abs(motor6);
    pwm.setPWM(10, 0, 0);
    pwm.setPWM(11, 0, motor6b);
    } 
    else
    {
    pwm.setPWM(10, 0, 0);  
    pwm.setPWM(11, 0, 0);
    }

    //right hip

    Input7 = pot7;
    //Setpoint7 = 0;
    PID7.Compute();

    Input8 = pot8;
    //Setpoint8 = 0;
    PID8.Compute();

    motor7 = (Output7 + Output8);
    motor8 = (Output7 - Output8); 

    if (motor7 < deadspot2)                                      
    {
    motor7a = abs(motor7);
    pwm.setPWM(12, 0, motor7a);
    pwm.setPWM(13, 0, 0);
    }
    else if (motor7 > deadspot)                         
    { 
    motor7b = abs(motor7);
    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, motor7b);
    } 
    else
    {
    pwm.setPWM(12, 0, 0);  
    pwm.setPWM(13, 0, 0);
    }


    if (motor8 < deadspot2)                                      
    {
    motor8a = abs(motor8);
    pwm.setPWM(14, 0, motor8a);
    pwm.setPWM(15, 0, 0);
    }
    else if (motor8 > deadspot)                         
    { 
    motor8b = abs(motor8);
    pwm.setPWM(14, 0, 0);
    pwm.setPWM(15, 0, motor8b);
    } 
    else
    {
    pwm.setPWM(14, 0, 0);  
    pwm.setPWM(15, 0, 0);
    }    
  


    } //end of main timed event

} // end of main loop




