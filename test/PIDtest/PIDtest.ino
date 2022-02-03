#define DEB

#include <PID_v1.h>
#include <Servo.h>

#define SAMPLERATE_DELAY_MS 500 //how often to read data from the board [milliseconds]

#define servoPin  10

//aggressive PID parameters
//double aggKi=0.2,aggKp=4,aggKd=1;
double aggKi=5,aggKp=2,aggKd=3;
//conservative PID parameters
double consKi=0.05,consKp=1,consKd=0.25;

double targetHeights[3]={700,850,1000};//target heights in mm
double sensThresholds[3]={50,75,100};//sensibility thresholds in mm (how much distance from the target to change PID parameters)
double targetHeight=targetHeights[2];
uint16_t sensThreshold=sensThresholds[0];

double measuredHeight, rotationAngle;//PID input and output

volatile unsigned long startT1,startT2,startT3;
volatile float distance1=0,distance2=0,distance3=0;

const byte heightId[4] = {9,133,241,194};  //TARGET HEIGHT TAG
const byte sensId[4] = {20,110,103,43};  //TARGET SENS TAG

double measuredHeights[10] = {650,680,750,800,400,320,560,780,800,900};


Servo srv;


PID myPID(&measuredHeight, &rotationAngle, &targetHeight, consKp, consKi, consKd, DIRECT);

void setup() {
    Serial.begin(115200);

    #ifdef SERVO
      init_servo();
    #endif

    myPID.SetMode(AUTOMATIC);
    //myPID.SetOutputLimits(0, 20);//10-13 degrees down, 8-9 degrees up
    myPID.SetOutputLimits(0, 60);
    myPID.SetSampleTime(SAMPLERATE_DELAY_MS);

    #ifdef DEB
      Serial.println("SETUP DONE");
    #endif
}
    
void loop() {
    static uint32_t timer=0;
    static uint8_t heightCfgCounter=0,sensCfgCounter=0, noCard=0, resetFlagSense=0, resetFlagHeight=0;
    static double gap;
    static uint8_t sCnt1=0;
    static int i=0;
    
    if (millis() - timer > SAMPLERATE_DELAY_MS) {
      timer = millis();//reset the timer

      //measuredHeight = (((distance2+distance3)/2)+distance1)/2;
      //measuredHeight = targetHeight-10;
      measuredHeight = measuredHeights[(i++)%10];
      
      gap = abs(targetHeight-measuredHeight);//distance away from target
      if (gap < sensThreshold){//close to target, use conservative tuning parameters
          myPID.SetTunings(consKp, consKi, consKd);
      }else{//far from target, use aggressive tuning parameters
          myPID.SetTunings(aggKp, aggKi, aggKd);
      }
      myPID.Compute();      
      Serial.println(rotationAngle);
      Serial.println(GetError());
        //srv.write(rotationAngle); 
    }
}

void init_servo(){    
    srv.attach(servoPin);
}
