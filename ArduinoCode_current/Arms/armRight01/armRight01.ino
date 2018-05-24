// Right Arm

#include <EasyTransfer.h>

EasyTransfer ET; 

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
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

RECEIVE_DATA_STRUCTURE mydata;


#include <Servo.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

int rot_demand = 0;
int lift_demand = 300;
int rot2_demand = 90;
int elb_demand = 90;

int rot_smoothed;
int lift_smoothed;
int rot2_smoothed;
int elb_smoothed;

int potRot;
int potLift;

float filterVal;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)

unsigned long count;

unsigned long previousMillis = 0;
const long interval = 10;

double Pk1 = 7;  //speed it gets there
double Ik1 = 2;
double Dk1 = 0;
double Setpoint1, Input1, Output1, Output1a;

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

double Pk2 = 7;  //speed it gets there
double Ik2 = 2;
double Dk2 = 0;
double Setpoint2, Input2, Output2, Output2a;

PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo


void setup() {

  Serial.begin(115200);

  ET.begin(details(mydata), &Serial);

  pinMode(3, OUTPUT);  // main rotation
  pinMode(5, OUTPUT);  // main rotation
  pinMode(6, OUTPUT);  // lift out
  pinMode(11, OUTPUT); // lift out

  pinMode(A0, INPUT);  // main rotation
  pinMode(A1, INPUT);  // lift out
  
  myservo1.attach(9);  // rotation
  myservo2.attach(10);  // elbow
  myservo1.write(90);
  myservo2.write(90);

   PID1.SetMode(AUTOMATIC);
   PID1.SetOutputLimits(-255, 255);
   PID1.SetSampleTime(10);

   PID2.SetMode(AUTOMATIC);
   PID2.SetOutputLimits(-255, 255);
   PID2.SetSampleTime(10);

  
}

int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

void loop() {

unsigned long currentMillis = millis();
if (currentMillis - previousMillis >= interval) {  //start timed event
  previousMillis = currentMillis;

  count += 1;

  potRot = analogRead(A0);
  potLift = analogRead(A1);

  if(ET.receiveData()){


        rot_demand = mydata.rot1_left;
        lift_demand = mydata.lift_left;
        rot2_demand = mydata.rot2_left;
        elb_demand = mydata.elbow_left;

}

  rot_demand = constrain(rot_demand,-200,500);
  lift_demand = constrain(lift_demand, -180,300);
  elb_demand = constrain(elb_demand,40,180);
  rot2_demand = constrain(rot2_demand,5,175);

  filterVal = 0.97;
  
  rot_smoothed =  smooth(rot_demand, filterVal, rot_smoothed);
  lift_smoothed =  smooth(lift_demand, filterVal, lift_smoothed);
  elb_smoothed =  smooth(elb_demand, filterVal, elb_smoothed);
  rot2_smoothed =  smooth(rot2_demand, filterVal, rot2_smoothed);
  
  myservo1.write(rot2_smoothed);
  myservo2.write(elb_smoothed);


  //*****************main rotation****************//
  
  Input1 = potRot-512;
  Setpoint1 = rot_smoothed;
  PID1.Compute();

      if (Output1 < 0) {
        Output1a = abs(Output1);
        analogWrite(5,Output1a);
        analogWrite(3,0);
      
      }
        else if (Output1 > 0) {
        Output1a = abs(Output1);
        analogWrite(3,Output1a);
        analogWrite(5,0);
      
      }
      else {
        analogWrite(3,0);
        analogWrite(5,0);
      }

  //*****************lift out****************//

  Input2 = potLift-512;
  Setpoint2 = lift_smoothed;
  PID2.Compute();

      if (Output2 < 0) {
        Output2a = abs(Output2);
        analogWrite(11,Output2a);
        analogWrite(6,0);
      
      }
        else if (Output2 > 0) {
        Output2a = abs(Output2);
        analogWrite(6,Output2a);
        analogWrite(11,0);
      
      }
      else {
        analogWrite(6,0);
        analogWrite(11,0);
      }



}  // end of timed event

            


}
