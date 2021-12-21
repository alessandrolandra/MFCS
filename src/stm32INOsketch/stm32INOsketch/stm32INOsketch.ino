#define DEB

#include <PID_v1.h>
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

#define SAMPLERATE_DELAY_MS 500 //how often to read data from the board [milliseconds]
#define MFRC_PRESCALER 2; //counter value to read MFRC only 1@s

#define UTR1 //comment if ultrasonic sensor 1 is not present
#define trigPin1  33
#define echoPin1  32
#define UTR2 //comment if ultrasonic sensor 2 is not present
#define trigPin2  25
#define echoPin2  26
#define UTR3 //comment if ultrasonic sensor 3 is not present
#define trigPin3  27
#define echoPin3  14

#define SERVO //comment if servo is not present
#define servoPin 12

#define MFRC_SDA 10 // SS on the board
#define MFRC_RST 9

MFRC522 mfrc522(MFRC_SDA, MFRC_RST);

//aggressive PID parameters
double aggKi=0.2,aggKp=4,aggKd=1;
//conservative PID parameters
double consKi=0.05,consKp=1,consKd=0.25;

double targetHeights[3]={700,850,1000};//target heights in mm
double sensThresholds[3]={575,100};//sensibility thresholds in mm (how much distance from the target to change PID parameters)
double targetHeight=targetHeights[0];
uint16_t sensThreshold=sensThresholds[0];

double measuredHeight, rotationAngle;//PID input and output

volatile unsigned long startT1,startT2,startT3;
volatile float distance1=0,distance2=0,distance3=0;

const byte heightId[4] = {9,133,241,194};  //TARGET HEIGHT TAG
const byte sensId[4] = {20,110,103,43};  //TARGET SENS   TAG

Servo srv;
PID myPID(&measuredHeight, &rotationAngle, &targetHeight, consKp, consKi, consKd, DIRECT);

void setup() {
    Serial.begin(115200);
  
    #ifdef UTR1
      pinMode(trigPin1, OUTPUT);
      pinMode(echoPin1, INPUT);
    #endif
    #ifdef UTR2
      pinMode(trigPin2, OUTPUT);  
      pinMode(echoPin2, INPUT);
    #endif
    #ifdef UTR3
      pinMode(trigPin3, OUTPUT);  
      pinMode(echoPin3, INPUT);
    #endif

    SPI.begin();  //enable SPI
    mfrc522.PCD_Init();    // Initialize MFRC522 Hardware
    
    #ifdef DEB
      Serial.println("SETUP DONE");
    #endif

    #ifdef UTR1
      triggerMeasure1();
    #endif
    #ifdef UTR2
      triggerMeasure2();
    #endif
    #ifdef UTR3
      triggerMeasure3();
    #endif

    #ifdef SERVO
      init_servo();
    #endif

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 20);//10-13 degrees down, 8-9 degrees up
}
    
void loop() {
    static uint32_t timer=0;
    static uint8_t mfrcCounter=0,heightCfgCounter=0,sensCfgCounter=0;
    static double gap;
    if (millis() - timer > SAMPLERATE_DELAY_MS) {
      timer = millis();//reset the timer

      #ifdef UTR1
        Serial.print("distance1: ");
        Serial.println(distance1);
        triggerMeasure1();
      #endif
      #ifdef UTR2
        Serial.print("distance2: ");
        Serial.println(distance2);
        triggerMeasure2();
      #endif
      #ifdef UTR3
        Serial.print("distance3: ");
        Serial.println(distance3);
        triggerMeasure3();
      #endif
      measuredHeight = (((distance2+distance3)/2)+distance1)/2;
      Serial.print("height: ");
      Serial.println(measuredHeight);
      gap = abs(targetHeight-measuredHeight);//distance away from target
      if (gap < sensThreshold){//close to target, use conservative tuning parameters
          myPID.SetTunings(consKp, consKi, consKd);
      }else{//far from target, use aggressive tuning parameters
          myPID.SetTunings(aggKp, aggKi, aggKd);
      }
      myPID.Compute();
      #ifdef SERVO        
        srv.write(rotationAngle);
      #endif

      mfrcCounter++;
      if(mfrcCounter == 2){
        if(!readMFRC(&heightCfgCounter,&sensCfgCounter)){
          heightCfgCounter=0;
          sensCfgCounter=0;
        }
        mfrcCounter=0;
      }      
    }
}

void triggerMeasure(int8_t trig_pin, int8_t echo_pin, void (*start_count_procedure)()){
    // Clears the trigPin
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    attachInterrupt(echo_pin, start_count_procedure, RISING);
}
void triggerMeasure1(){
    triggerMeasure(trigPin1,echoPin1,startCount1);
}
void triggerMeasure2(){
    triggerMeasure(trigPin2,echoPin2,startCount2);
}
void triggerMeasure3(){
    triggerMeasure(trigPin3,echoPin3,startCount3);
}

void startCount(volatile unsigned long *start_time,int8_t echo_pin,void (*stop_count_procedure)()){
    *start_time = micros();
    attachInterrupt(echo_pin, stop_count_procedure, FALLING);
}
void startCount1(){
    startCount(&startT1,echoPin1,stopCount1);
}
void startCount2(){
    startCount(&startT2,echoPin2,stopCount2);
}
void startCount3(){
    startCount(&startT3,echoPin3,stopCount3);
}

void stopCount(volatile int64_t start_time,volatile float *distance,int8_t echo_pin){
    unsigned long durationMicros = micros()-start_time;
    //the speed of sound, in air, at 20 degrees C is 343m/s... half distance in mm: micros/10^6*343*10^3 /2 = micros*0.1715
    *distance = durationMicros*0.1715;
    /*if(!(*distance>=230 && *distance<=4000)){
      *distance=-1;
    }*/
    detachInterrupt(echo_pin);    
}
void stopCount1(){
    stopCount(startT1,&distance1,echoPin1);
}
void stopCount2(){
    stopCount(startT2,&distance2,echoPin2);
}
void stopCount3(){
    stopCount(startT3,&distance3,echoPin3);
}

void init_servo(){
    srv.attach(servoPin); 
}

int readMFRC(uint8_t *heightCfgCounter,uint8_t *sensCfgCounter){
    // Look for new cards
    if (mfrc522.PICC_IsNewCardPresent()){
        // Select one of the cards
        if (mfrc522.PICC_ReadCardSerial()){
          checkCardTag(heightCfgCounter,sensCfgCounter);
          return 1;
        }
    }
    return 0;
}
void checkCardTag(uint8_t *heightCfgCounter,uint8_t *sensCfgCounter){
    if(memcmp(mfrc522.uid.uidByte,heightId,sizeof(heightId))==0){
        if(*heightCfgCounter<3){
          *heightCfgCounter++;
          targetHeight=targetHeights[*heightCfgCounter];
        }
    }else if(memcmp(mfrc522.uid.uidByte,sensId,sizeof(sensId))==0){
        if(*sensCfgCounter<3){
          *sensCfgCounter++;
          sensThreshold=sensThresholds[*sensCfgCounter];
        }
    }
}
