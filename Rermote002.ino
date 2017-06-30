#include <EasyTransfer.h>

//create object
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (addr, en, rw, rs, d4, d5, d6, d7, backlight, polarity)

int but1;
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

String count;
String IMU3a;
String IMU3b;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t buttons;   
    int16_t buttons2;  
    int16_t axis1;
    int16_t axis2;
    int16_t axis3;
};

struct RECEIVE_DATA_STRUCTURE_REMOTE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t IMU3a;
  int16_t IMU3b;
  int16_t stepTime;
  int16_t mode;
  int16_t count;
};

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE_REMOTE mydata_remote;

int state; // BT state

unsigned long previousMillis = 0;
const long interval = 40;

unsigned long previousDispMillis = 0;
const long Dispinterval = 10;

void setup() {

  lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight

  Serial.begin(57600);
  Serial3.begin(57600);

  ET1.begin(details(mydata_send), &Serial3);
  ET2.begin(details(mydata_remote), &Serial3);

  // NOTE: Cursor Position: (CHAR, LINE) starts at 0  
  lcd.setCursor(0,0);
  lcd.print("Robot Remote        ");
  lcd.setCursor(0,1);
  lcd.print("XRobots.co.uk       ");
  
  pinMode(27, INPUT_PULLUP);
  pinMode(29, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(35, INPUT_PULLUP);
  pinMode(37, INPUT_PULLUP);
  pinMode(39, INPUT_PULLUP);
  pinMode(41, INPUT_PULLUP);
  pinMode(43, INPUT_PULLUP);
  pinMode(45, INPUT_PULLUP);
  pinMode(47, INPUT_PULLUP);
  pinMode(49, INPUT_PULLUP);
  pinMode(51, INPUT_PULLUP);
  pinMode(53, INPUT_PULLUP);

  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  pinMode(3, INPUT); // BT state
  pinMode(2, OUTPUT); // LED

  digitalWrite(2, LOW); // turn off LED

}

  void pair() {
    state = digitalRead(3);
    while(state == 0) {
        lcd.setCursor(0,2);
        lcd.print("Waiting to Pair BT  ");
        lcd.setCursor(0,3);
        lcd.print("               ");
        state = digitalRead(3);
        digitalWrite(2,HIGH);
        delay(250);
        digitalWrite(2,LOW);
        delay(250);    
    }

    delay(1000);  // wait before sending data
    digitalWrite(2,HIGH); 

}

void loop() {


 unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
    previousMillis = currentMillis;



// check to see if BT is paired
state = digitalRead(3);
if (state == 0) {
  pair();
}
else {
lcd.setCursor(0,2);
lcd.print("Paired to Robot!    ");
}


but1 = ~ digitalRead(39) +2;
but2 = ~ digitalRead(41) +2;
but3 = ~ digitalRead(43) +2;
but4 = ~ digitalRead(37) +2;
but5 = ~ digitalRead(45) +2;
but6 = ~ digitalRead(35) +2;
but7 = ~ digitalRead(47) +2;
but8 = ~ digitalRead(33) +2;
but9 = ~ digitalRead(49) +2;
but10 = ~ digitalRead(31) +2;
but11 = ~ digitalRead(51) +2;
but12 = ~ digitalRead(29) +2;
but13 = ~ digitalRead(53) +2;
but14 = ~ digitalRead(27) +2;

mbut1 = ~ digitalRead(6) +2;
mbut2 = ~ digitalRead(7) +2;

mydata_send.axis1 = analogRead(A1)-492;
mydata_send.axis2 = analogRead(A0)-516;
mydata_send.axis3 = analogRead(A2)-522;
mydata_send.buttons = 256;

mydata_send.buttons = 0;
mydata_send.buttons2 = 0;

if (but1 == 1) {
  mydata_send.buttons +=1;
}
if (but2 == 1) {
  mydata_send.buttons +=2;
}
if (but3 == 1) {
  mydata_send.buttons +=4;
}
if (but4 == 1) {
  mydata_send.buttons +=8;
}
if (but5 == 1) {
  mydata_send.buttons +=16;
}
if (but6 == 1) {
  mydata_send.buttons +=32;
}
if (but7 == 1) {
  mydata_send.buttons +=64;
}
if (but8 == 1) {
  mydata_send.buttons +=128;
}
if (but9 == 1) {
  mydata_send.buttons +=256;
}
if (but10 == 1) {
  mydata_send.buttons +=512;
}
if (but11 == 1) {
  mydata_send.buttons +=1024;
}
if (but12 == 1) {
  mydata_send.buttons +=2048;
}
if (but13 == 1) {
  mydata_send.buttons +=4096;
}
if (but14 == 1) {
  mydata_send.buttons +=8192;
}

if (mbut1 == 1) {
  mydata_send.buttons2 +=1;
}
if (mbut2 == 1) {
  mydata_send.buttons2 +=2;
}


ET1.sendData();

 } // end of timed event send


if (currentMillis - previousDispMillis >= Dispinterval) {  // start timed event for read and send
  previousDispMillis = currentMillis;

  

        if(ET2.receiveData()){
          count = String(mydata_remote.count);
          IMU3a = String(mydata_remote.IMU3a);
          IMU3b = String(mydata_remote.IMU3b);
          lcd.setCursor(0,3);
          lcd.print(count);
          lcd.setCursor(7,3);
          lcd.print(IMU3a);
          lcd.setCursor(13,3);
          lcd.print(IMU3b);

          if (mydata_remote.mode == 0) {
            lcd.setCursor(0,0);
            lcd.print("Mode 0 - Slow Walk  ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }

          if (mydata_remote.mode == 1) {
            lcd.setCursor(0,0);
            lcd.print("Mode 1 - Fast Walk  ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 2) {
            lcd.setCursor(0,0);
            lcd.print("Mode 2 - Arm Demo   ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 3) {
            lcd.setCursor(0,0);
            lcd.print("Mode 3 - Arm Demo 2 ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 4) {
            lcd.setCursor(0,0);
            lcd.print("Mode 4              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 5) {
            lcd.setCursor(0,0);
            lcd.print("Mode 5              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          

        }

        
}  // end of second timed event






}
