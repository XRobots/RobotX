//RobotX code using foot pressure sensors

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EasyTransfer.h>
#include <EasyTransferI2C.h>

EasyTransfer ET1; 
EasyTransferI2C ET2;
EasyTransfer ET2a; 
EasyTransfer ET3; 
EasyTransfer ET4; // left leg/foot
EasyTransfer ET5; // right leg/foot

struct SEND_DATA_STRUCTURE{                                               // send data to arms
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t rot1_left;
  int16_t lift_left;
  int16_t rot2_left;
  int16_t elbow_left;
  int16_t rot1_right;
  int16_t lift_right;
  int16_t rot2_right;
  int16_t elbow_right;
};

struct SEND_DATA_STRUCTURE_IS{                                            // send data to spiderman peripherals
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t pot1 = 512;
  int16_t pot2 = 512;
  int16_t pot3 = 512;
  int16_t but1; 
  int16_t but2; 
  int16_t but3; 
  int16_t but4; 
};

struct SEND_DATA_STRUCTURE_REMOTE{                                        // send data back to remote display
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t IMU3a;
  int16_t IMU3b;
  int16_t stepTime;
  int16_t mode;
  int16_t count;
};

struct RECEIVE_DATA_STRUCTUREI2C{                                       // receive data from remote over i2c
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t buttons;   
    int16_t buttons2;   
    int16_t axis1;    
    int16_t axis2;
    int16_t axis3;

};

struct RECEIVE_DATA_STRUCTURE_IMU1{                                       // receive data from left foot/leg
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    float IMU1;   
    int16_t sens1;   
    int16_t sens2;    
    int16_t sens3;
    int16_t sens4;
};

struct RECEIVE_DATA_STRUCTURE_IMU2{                                       // receive data from right foot/leg
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    float IMU2;   
    int16_t sens1;   
    int16_t sens2;    
    int16_t sens3;
    int16_t sens4;
};


RECEIVE_DATA_STRUCTUREI2C mydata_recI2C ;

#define I2C_SLAVE_ADDRESS 9

SEND_DATA_STRUCTURE_REMOTE mydata_remote;

SEND_DATA_STRUCTURE_IS mydata_IS;

SEND_DATA_STRUCTURE mydata_arms;

//---- IMU and foot pressure sensing below

RECEIVE_DATA_STRUCTURE_IMU1 mydata_IMU1;
RECEIVE_DATA_STRUCTURE_IMU2 mydata_IMU2;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define OFFSET1  2090;  //  2100 right ankle - higher value tips robot inwards
#define OFFSET2  2000;  //  2030 right lower leg - lower value is more bent
#define OFFSET3  2285;  //  2315 right upper leg - lower value is more bent
#define OFFSET4  2320;  //  2280 (*replaced pot*) right hip - higher moves in
#define OFFSET5  2450;  //  2450 left ankle - higher value tips robot inwards
#define OFFSET6  2180;  //  left lower leg
#define OFFSET7  2380;  //  left upper leg
#define OFFSET8  2115;  //  2145  left hip - higher moves in

#define OFFSET9  2150;   // front back body pot  - higher number back it move back

int deadspot = 0;
int deadspot2 = 0;

int remote;
int count = 0;

int but1;  // remote buttons
int but2;
int but3;
int but4;
int but5;
int but6;
int but7;
int but8;
int but9;
int but10;
int but11;
int but12;
int but13;
int but14;

int mbut1;
int mbut2;
int mode;
int modedb;

int pot1;  // joint positions
int pot2;
int pot3;
int pot4;
int pot5;
int pot6;
int pot7;
int pot8;

int pot9; // front back body action

int fsr1;   // right foot
int fsr1a;  // threshold
int fsr2;   // left foot
int fsr2a;  // threshhold

int volume; // motor max speed knob

int state01 = 0;
int stateLeft = 0;
int stateRight = 0;
int straightLeft = 0;
int straightRight = 0;
int stateLeft2 = 0;
int stateRight2 = 0;

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

unsigned long previousDispMillis = 0;
const long Dispinterval = 40;

unsigned long previousArmMillis = 0;
const long Arminterval = 20;

unsigned long previousHeadMillis = 0;
const long Headinterval = 20;

unsigned long previousFSR1Millis = 0;
unsigned long previousFSR2Millis = 0;

unsigned long previousFSR1aMillis = 0;
unsigned long previousFSR2aMillis = 0;

long FSR1Interval;
long FSR2Interval;

int fsr1flag;
int fsr2flag;

unsigned long previousStepMillis = 0;

double IMUtrack01;
double IMUtrack01_av;

const int numReadings = 500;

double readings[numReadings];      // the readings from the analog input    // IMUtrack01 running average
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average

int number1 = 0;
int number4 = 0;
int number5 = 0;
int number8 = 0;


double Pk1 = 5;   // right ankle
double Ik1 = 0;
double Dk1 = 0.2;

double Pk2 = 20; 
double Ik2 = 0;
double Dk2 = 0.2;

double Pk3 = 20; 
double Ik3 = 0;
double Dk3 = 0.2;

double Pk4 = 10;   // right hip
double Ik4 = 0;
double Dk4 = 0.2;

double Pk5 = 5;  // left ankle
double Ik5 = 0;
double Dk5 = 0.2;

double Pk6 = 20; 
double Ik6 = 0;
double Dk6 = 0.2;

double Pk7 = 20; 
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

double Pka2 = 10;   // IMU 1 - left leg ankle
double Ika2 = 0;
double Dka2 = 0.1;

double Pkb2 = 10;   // IMU 2 - right leg ankle
double Ikb2 = 0;
double Dkb2 = 0.1;

double Pkc = 5;   // IMU 3 - body side to side
double Ikc = 0;
double Dkc = 0.1;

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
  Serial2.begin(57600);
  Serial3.begin(115200);

  ET1.begin(details(mydata_arms), &Serial1);  // arms
  
  ET2a.begin(details(mydata_remote), &Serial2);  // remote

  ET3.begin(details(mydata_IS), &Serial3);  // IS head

  Wire.begin(I2C_SLAVE_ADDRESS);
  
  ET2.begin(details(mydata_recI2C), &Wire);  // //remote bridge
  Wire.onReceive(receive);

  //----- IMU and foot pressure sensing 

  ET4.begin(details(mydata_IMU1), &Serial1); // left foot & IMU
  ET5.begin(details(mydata_IMU2), &Serial2); // left foot & IMU

  pinMode(13, OUTPUT);  // LED  

  mydata_arms.rot1_left = 0;
  mydata_arms.lift_left = 300;
  mydata_arms.rot2_left = 90;
  mydata_arms.elbow_left = 90;
  mydata_arms.rot1_right = 0;
  mydata_arms.lift_right = 300;
  mydata_arms.rot2_right = 90;
  mydata_arms.elbow_right = 90;
  
  
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
  PID1.SetOutputLimits(-800, 800);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              // PID Setup - right lower leg
  PID2.SetOutputLimits(-2500, 2500);
  PID2.SetSampleTime(10);

  PID3.SetMode(AUTOMATIC);              // PID Setup - right upper leg
  PID3.SetOutputLimits(-2500, 2500);
  PID3.SetSampleTime(10);

  PID4.SetMode(AUTOMATIC);              // PID Setup - right hip
  PID4.SetOutputLimits(-800, 800);
  PID4.SetSampleTime(10);

  PID5.SetMode(AUTOMATIC);              // PID Setup - left ankle
  PID5.SetOutputLimits(-800, 800);
  PID5.SetSampleTime(10);

  PID6.SetMode(AUTOMATIC);              // PID Setup - left lower leg
  PID6.SetOutputLimits(-2500, 2500);
  PID6.SetSampleTime(10);

  PID7.SetMode(AUTOMATIC);              // PID Setup - left upper leg
  PID7.SetOutputLimits(-2500, 2500);
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


    //***************** remote handling ****************

      if(ET2.receiveData()){
    //this is how you access the variables. [name of the group].[variable name]
    //since we have data, we will blink it out. 

    but1 = bitRead(mydata_recI2C.buttons, 0);
    but2 = bitRead(mydata_recI2C.buttons, 1);
    but3 = bitRead(mydata_recI2C.buttons, 2);
    but4 = bitRead(mydata_recI2C.buttons, 3);
    but5 = bitRead(mydata_recI2C.buttons, 4);
    but6 = bitRead(mydata_recI2C.buttons, 5);
    but7 = bitRead(mydata_recI2C.buttons, 6);
    but8 = bitRead(mydata_recI2C.buttons, 7);
    but9 = bitRead(mydata_recI2C.buttons, 8);
    but10 = bitRead(mydata_recI2C.buttons, 9);
    but11 = bitRead(mydata_recI2C.buttons, 10);
    but12 = bitRead(mydata_recI2C.buttons, 11);
    but13 = bitRead(mydata_recI2C.buttons, 12);
    but14 = bitRead(mydata_recI2C.buttons, 13);

    mbut1 = bitRead(mydata_recI2C.buttons2, 0);
    mbut2 = bitRead(mydata_recI2C.buttons2, 1);

  }  // end of receive data   

 if (currentMillis - previousDispMillis >= Dispinterval) {  //start timed event for Remote MCD transmission
    previousDispMillis = currentMillis;


  count = count+1;
  if (count >= 999) {
    count = 0;
  }

    if (mbut2 == 1 && modedb == 0){
      mode = mode -1;
      modedb = 1;
    }
    else if (mbut1 == 1 && modedb == 0) {
      mode = mode +1;
      modedb = 1;
    }
    if (mbut2 == 0 && mbut1 == 0){
      modedb = 0;   
    }

    
  mode = constrain(mode,0,5);

  mydata_remote.count = count;
  mydata_remote.IMU3a = (int) IMU3a;
  mydata_remote.IMU3b = (int) IMU3b;
  mydata_remote.mode = mode;

  ET2a.sendData();  // send data back to remote

 }  // end timed event for remote LCD



 if(ET4.receiveData()) {
  /*
    Serial.print(mydata_IMU1.IMU1);
    Serial.print(" , ");
    Serial.print(mydata_IMU1.sens1);
    Serial.print(" , ");
    Serial.print(mydata_IMU1.sens2);
    Serial.print(" , ");
    Serial.print(mydata_IMU1.sens3);
    Serial.print(" , ");
    Serial.print(mydata_IMU1.sens4);
    Serial.print(" , ");
   */
 }


  if(ET5.receiveData()) {
    /*
    Serial.print(mydata_IMU2.IMU2);
    Serial.print(" , ");
    Serial.print(mydata_IMU2.sens1);
    Serial.print(" , ");
    Serial.print(mydata_IMU2.sens2);
    Serial.print(" , ");
    Serial.print(mydata_IMU2.sens3);
    Serial.print(" , ");
    Serial.println(mydata_IMU2.sens4);
    */

    mydata_IMU2.IMU2 = mydata_IMU2.IMU2 + 1; // correct for fitting angle
 }



 

    if (Serial3.available() > 0) {  // IMU 3 - body
            // read the incoming byte:
            IMU3a = Serial3.parseFloat();
            IMU3b = Serial3.parseFloat();
            IMU3a = IMU3a + 0.7;        //front to back
            IMU3b = IMU3b +1.9;         // 1.8 Side to side                 
            if (Serial3.read() == '\n') {        
            }
    }

    
    fsr2 = (mydata_IMU2.sens1 + mydata_IMU2.sens2 + mydata_IMU2.sens3 + mydata_IMU2.sens4);
    fsr1 = (mydata_IMU1.sens1 + mydata_IMU1.sens2 + mydata_IMU1.sens3 + mydata_IMU1.sens4);

    // work out how long it was since the foot hit the ground

    Serial.print(fsr1);
    Serial.print(" , ");
    Serial.println(fsr2);


    // right foot

    if (fsr1 < 900 && fsr1flag == 0) {
      previousFSR1Millis = currentMillis;
      fsr1flag = 1;
    }

    FSR1Interval = currentMillis - previousFSR1Millis;


    if (fsr1 > 900 && fsr1flag == 1) {
      fsr1flag = 0;
      FSR1Interval = 0;
    }

    // left foot
    


    if (fsr2 < 900 && fsr2flag == 0) {
      previousFSR2Millis = currentMillis;
      fsr2flag = 1;
    }

    FSR2Interval = currentMillis - previousFSR2Millis;

    if (fsr2 > 900 && fsr2flag == 1) {
      fsr2flag = 0;
      FSR2Interval = 0;
    }   
 
  
    Inputc = IMU3b;
    
    PIDc.Compute();

   IMUtrack01 = (mydata_IMU1.IMU1 + mydata_IMU2.IMU2);

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

        if (state01 == 0 && stateRight == 0 && straightRight == 3  && currentMillis - previousStepMillis >= 390 && FSR2Interval > 80) { 
            previousStepMillis = currentMillis;
            if (but7 == 0) {
              Setpointc = 0;
            }
            else if (but7 == 1) {
              Setpointc = 0;
            }
            stateLeft = 1;
            state01 = 1;
              }          

        if (state01 == 1 && stateLeft == 0 && straightLeft == 3 &&   currentMillis - previousStepMillis >= 390 && FSR1Interval > 80) { 
            previousStepMillis = currentMillis;
            if (but7 == 0) {
              Setpointc = 0;
            }
            else if (but7 == 1) {
              Setpointc = 0;
            }
            stateRight = 1;                
            state01 = 0;
              }


        }    // end of state machine1

    
                                   

    //******* Leg lifting ************

    if (but7 == 0) {                                                        //************** normal stepping ************************

    if (stateLeft == 0) {  
      
                  if (but2 == 1){                                           // if the state machine is on
                      Setpoint2 = 0;// - fbStab;;
                      Setpoint3 = 0 + fbStab;
                    if (but9 == 1) {
                        Setpoint2 = -35;
                        Setpoint3 = 35 + fbStab;
                    }

                  }
                  else if(but2 == 0){                                       // if the state machine is off
                      Setpoint2 = 0 - fbStab;
                      Setpoint3 = 0 + fbStab;
                      Setpointc = 0;

                  }
    }
        else if (stateLeft == 1) {                                          // leg is triggered from state machine
                  Setpoint2 = -270;
                  Setpoint3 = -270;
                  stateLeft = 2;
                  straightLeft = 0;
            }

              if (stateLeft == 2 && Input2 < -200 && Input3 < -200) {  // if leg is bent
                  stateLeft = 3;
              }

              if (stateLeft == 3) {                            // wait to cross point before bringing leg down
                stateLeft = 0;
              }                            
           
              if (but9 == 0) {
                  if (Input2 > -60 && Input3 > -60 && straightLeft == 0) {      // if leg is straight
                    straightLeft = 3;                                            // set straight flag
                  }
              }
              else if (but9 == 1){
                   if (Input2 > -75 && Input3 < 0 && straightLeft == 0) {      // if leg is straight
                    straightLeft = 3;                                            // set straight flag
                  }
              }




    if (stateRight == 0) {
                if (but2 == 1) {                                       // if the state machine is on
                    Setpoint6 = 0;// - fbStab;
                    Setpoint7 = 0 + fbStab;
                    if (but9 == 1) {
                        Setpoint6 = -35;
                        Setpoint7 = 35 + fbStab;
                    }
                }                                                       // if the state machine is off
                else if (but2 == 0) {
                    Setpoint6 = 0 - fbStab;
                    Setpoint7 = 0 + fbStab;
                    Setpointc = 0;

                }

      }
          
          else if (stateRight == 1) {                                     // leg is triggered from state machine
                Setpoint6 = -270;
                Setpoint7 = -270;
                stateRight = 2;
                straightRight =0;
                }
                
                if (stateRight == 2 && Input6 < -200 && Input7 < -200) {  // if leg is bent
                  stateRight = 3;
                }

                if (stateRight == 3) {                       // wait to cross point before beinging leg down
                  stateRight = 0;
                }

                if (but9 == 0) { 
                    if (Input6 > -60 && Input7 > -60 && straightRight == 0) {   // if leg is straight
                      straightRight = 3;                                        // set straight flag
                    } 
                }
                else if (but9 == 1){
                     if (Input6 > -75 && Input7 < 0 && straightRight == 0) {   // if leg is straight
                      straightRight = 3;                                        // set straight flag
                    } 

                }



                
    }           //******* Leg lifting End ************
    

    Inputa = mydata_IMU1.IMU1;
    Inputb = mydata_IMU2.IMU2;
    Inputa2 = mydata_IMU1.IMU1;
    Inputb2 = mydata_IMU2.IMU2;

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

          mydata_arms.rot1_left = 200;
          mydata_arms.rot1_right = -200;
        }
    
        else if (but5 == 1) {
          Setpoint1 = 150;
          Setpoint5 = -150;
          Setpoint4 = -150;
          Setpoint8 = 150;   

          mydata_arms.rot1_left = -200;
          mydata_arms.rot1_right = 200;
        }
    
        else if (but6 == 0 && but5 == 0) {
          Setpoint1 = 0;
          Setpoint5 = 0;
          Setpoint8 = 0;
          Setpoint4 = 0;
          
          mydata_arms.rot1_left = 0;
          mydata_arms.rot1_right = 0;
          
          
        }

    }

      else if (but1 == 1) {           // side to side stability 

            if (but3 == 0 && but4 == 0) {
              Setpointc = 0;              
            }

            else if (but4 == 1) {
              Setpointc = 5;

            }

            else if (but3 == 1){
               Setpointc = -5;
                     
            }

            
            Setpoint1 = 0 + constrain((Outputc/2.5),-180,180); // ankle
            Setpoint5 = 0 - constrain((Outputc/2.5),-180,180);  // ankle
            Setpoint8 = 0 + constrain(Outputc,-180,180);   
            Setpoint4 = 0 - constrain(Outputc,-180,180);   
            
            
                                
      }                               // end of dynamnic mode

    //******************** arm control *****************    

    if (mode == 2) {   
        mydata_arms.rot1_left = mydata_recI2C.axis1;            // move arms
        mydata_arms.rot1_right = mydata_recI2C.axis1*-1;
    
        mydata_arms.lift_left = 100-mydata_recI2C.axis2;
        mydata_arms.lift_right = 100+mydata_recI2C.axis2;
    
        mydata_arms.rot2_left = 90+mydata_recI2C.axis3;
        mydata_arms.rot2_right = 90+mydata_recI2C.axis3;


        mydata_IS.pot1 = 0;                                     // keep head still
        mydata_IS.pot2 = 0;
        mydata_IS.pot3 = 0;
    }

    
   //******************** IS head control *****************
   
   else if (mode == 3) {

      mydata_IS.pot1 = mydata_recI2C.axis1;   // move head
      mydata_IS.pot2 = mydata_recI2C.axis2;
      mydata_IS.pot3 = mydata_recI2C.axis3;
      mydata_IS.but1 = but14; 


      mydata_arms.rot1_left = 0;              // keep arms still
      mydata_arms.rot1_right = 0;

      mydata_arms.lift_left = 250;
      mydata_arms.lift_right = 250;

      mydata_arms.rot2_left = 90;
      mydata_arms.rot2_right = 90;
   }



   //************** both arm and head control #1 ********************

   else if (mode == 4) {

        mydata_arms.rot1_left = mydata_recI2C.axis1;            // move arms
        mydata_arms.rot1_right = mydata_recI2C.axis1*-1;
    
        mydata_arms.rot2_left = 100-mydata_recI2C.axis2;
        mydata_arms.rot2_right = 100+mydata_recI2C.axis2;
    
        mydata_arms.elbow_left = 90+mydata_recI2C.axis3;
        mydata_arms.elbow_right = 90+mydata_recI2C.axis3;

        mydata_IS.pot1 = mydata_recI2C.axis1;   // move head
        mydata_IS.pot2 = mydata_recI2C.axis2;
        mydata_IS.pot3 = mydata_recI2C.axis3;
        mydata_IS.but1 = but14; 
        mydata_IS.but2 = but13; 
        mydata_IS.but3 = but12; 
        mydata_IS.but4 = but11;

  
 }

 //************** mode is neither head or arm control *************

   else {
      mydata_IS.pot1 = 0;             // keep head still
      mydata_IS.pot2 = 0;
      mydata_IS.pot3 = 0;

      mydata_arms.rot1_left = 0;      // leep arms still
      mydata_arms.rot1_right = 0;

      mydata_arms.lift_left = 250;
      mydata_arms.lift_right = 250;

      mydata_arms.rot2_left = 90;
      mydata_arms.rot2_right = 90;
  }


    //********************** send data to head and arms *************************

     if (currentMillis - previousArmMillis >= Arminterval) {  //start timed event to send data to arms
        previousArmMillis = currentMillis;
        ET1.sendData();   // send data to arms     
     }

     if (currentMillis - previousHeadMillis >= Headinterval) {  //start timed event to send data to IS head
        previousHeadMillis = currentMillis;
        ET3.sendData();   // send data to head     
     }
     

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

void receive(int numBytes) {}




