#include <EasyTransfer.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

//create object
EasyTransfer ET1;

EasyTransferI2C ET2;


struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t buttons; 
    int16_t buttons2;   
    int16_t axis1;
    int16_t axis2;
    int16_t axis3;
};

struct SEND_DATA_STRUCTUREI2C{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t buttons;  
    int16_t buttons2;      
    int16_t axis1;    
    int16_t axis2;
    int16_t axis3;
};
    
RECEIVE_DATA_STRUCTURE mydata_rec; // rec from remote

SEND_DATA_STRUCTUREI2C mydata_sendI2C;

#define I2C_SLAVE_ADDRESS 9

unsigned long previousMillis = 0;
const long interval = 20;

void setup() {

Serial.begin(57600);
ET1.begin(details(mydata_rec), &Serial);

Wire.begin();
ET2.begin(details(mydata_sendI2C), &Wire);

}

void loop() {

   unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {  //start timed event
    previousMillis = currentMillis;

    


        if(ET1.receiveData()){

          mydata_sendI2C.buttons = mydata_rec.buttons;
          mydata_sendI2C.buttons2 = mydata_rec.buttons2;

          mydata_sendI2C.axis1 = mydata_rec.axis1;
          mydata_sendI2C.axis2 = mydata_rec.axis2;
          mydata_sendI2C.axis3 = mydata_rec.axis3;
          ET2.sendData(I2C_SLAVE_ADDRESS);

        }

          
          }// end timed event     
       


    }  

