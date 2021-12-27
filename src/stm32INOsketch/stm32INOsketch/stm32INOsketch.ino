#define DEB
//#define ESP32 //uncomment to use ESP32
#define UTR1 //comment if ultrasonic sensor 1 is not present
//#define UTR2 //comment if ultrasonic sensor 2 is not present
//#define UTR3 //comment if ultrasonic sensor 3 is not present
//#define SERVO //comment if servo is not present
//#define MFRC //comment if MFRC is not present
#define BNO //comment if BNO is not present

#include <PID_v1.h>
#ifndef ESP32
  #include <Servo.h>
#endif
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_BNO08x.h>

#define SAMPLERATE_DELAY_MS 500 //how often to read data from the board [milliseconds]

#ifdef ESP32
  #define trigPin1  12
  #define echoPin1  13
#else
  #define trigPin1  PA_0
  #define echoPin1  PA_1
#endif
#ifdef ESP32
  #define trigPin2  14
  #define echoPin2  27
#else
  #define trigPin2  PA_2
  #define echoPin2  PA_3
#endif
#ifdef ESP32
  #define trigPin3  26
  #define echoPin3  25
#else
  #define trigPin3  PB_4
  #define echoPin3  PB_5
#endif

#ifdef ESP32
  #define servoPin 12
#else
  #define servoPin PA_8
#endif

#ifdef ESP32
  #define MFRC_SS 10
  #define MFRC_RST 9
#else
  #define MFRC_SS PA_6
  #define MFRC_RST PA_7
#endif

/*
 * MFRC ESP32 pinout
 * SS  10
 * RST 9
 * SDA 21  
 * SCL 22
 * 
 * MFRC STM32 pinout
 * SS  PA_6
 * RST PA_7
 * SDA PB_9  
 * SCL PB_8
*/
#ifdef MFRC
  MFRC522 mfrc522(MFRC_SS, MFRC_RST);
#endif

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
const byte sensId[4] = {20,110,103,43};  //TARGET SENS TAG

#ifdef SERVO
  Servo srv;
#endif

/*
 * BNO ESP32 pinout
 * SDA 21  
 * SCL 22
 * 
 * BNO STM32 pinout
 * SDA PB_3  
 * SCL PB_10
*/
#ifdef BNO
  Adafruit_BNO08x bno085(-1); // reset pin not required in I2C
#endif

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

    SPI.begin();
    #ifdef MFRC
      mfrc522.PCD_Init(); 
    #endif

    #ifdef BNO
      init_BNO();
    #endif

    #ifdef SERVO
      init_servo();
    #endif

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 20);//10-13 degrees down, 8-9 degrees up

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
}
    
void loop() {
    static uint32_t timer=0;
    static uint8_t heightCfgCounter=0,sensCfgCounter=0, noCard=0, resetFlagSense=0, resetFlagHeight=0;
    static double gap;
    static sh2_SensorValue_t sensorValue;
    
    if (millis() - timer > SAMPLERATE_DELAY_MS) {
      timer = millis();//reset the timer

      #ifdef UTR1
        //Serial.print("distance1: ");
        //Serial.println(distance1);
        triggerMeasure1();
      #endif
      #ifdef UTR2
        //Serial.print("distance2: ");
        //Serial.println(distance2);
        triggerMeasure2();
      #endif
      #ifdef UTR3
        //Serial.print("distance3: ");
        //Serial.println(distance3);
        triggerMeasure3();
      #endif

      #ifdef BNO
        if(bno085.wasReset()) {
          enableReports();
        }
        if(!bno085.getSensorEvent(&sensorValue)) {
          #ifdef DEB
            Serial.println("Cannot get sensor event");
          #endif
          return;
        }
        if(sensorValue.status < 2) {
          #ifdef DEB
            Serial.print("Low accuracy ");
            Serial.print(sensorValue.sensorId, HEX);
          #endif
          return;
        }
        if(sensorValue.sensorId == SH2_ROTATION_VECTOR){
          readRotation(sensorValue.un.rotationVector);
        }
      #endif
      
      measuredHeight = (((distance2+distance3)/2)+distance1)/2;
      //Serial.print("height: ");
      //Serial.println(measuredHeight);
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

      #ifdef MFRC
        readMFRC(&heightCfgCounter,&sensCfgCounter, &noCard, &resetFlagSense, &resetFlagHeight);
        if(noCard == 32){
          #ifdef DEB
            Serial.println("No card present");
          #endif
          resetFlagSense = 1;
          resetFlagHeight = 1;
          noCard = 0;
        }
      #endif
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

#ifdef SERVO
  void init_servo(){    
      srv.attach(servoPin);
  }
#endif

#ifdef MFRC
  void readMFRC(uint8_t *heightCfgCounter,uint8_t *sensCfgCounter, uint8_t *noCard, uint8_t *resetFlagSense, uint8_t *resetFlagHeight){
     // Select one of the cards
      mfrc522.PICC_IsNewCardPresent();
      if (mfrc522.PICC_ReadCardSerial()){
        checkCardTag(heightCfgCounter,sensCfgCounter, resetFlagSense, resetFlagHeight);
      }else{
        (*noCard)++;
      }
  }
  void checkCardTag(uint8_t *heightCfgCounter,uint8_t *sensCfgCounter, uint8_t *resetFlagSense, uint8_t *resetFlagHeight){
      if(memcmp(mfrc522.uid.uidByte,heightId,sizeof(heightId))==0){
          if(*resetFlagHeight){
            *heightCfgCounter = 0;
            *resetFlagHeight = 0;
          }
          Serial.println("Height card detected");
          Serial.print("Height counter: ");
          Serial.println(*heightCfgCounter);
          if(*heightCfgCounter<3){
            (*heightCfgCounter)++;
            targetHeight=targetHeights[*heightCfgCounter];
          }
      }else if(memcmp(mfrc522.uid.uidByte,sensId,sizeof(sensId))==0){
          if(*resetFlagSense){
            *sensCfgCounter = 0;
            *resetFlagSense = 0;
          }
          Serial.println("Sense card detected");
          Serial.print("Sense counter: ");
          Serial.println(*sensCfgCounter);
          if(*sensCfgCounter<3){
            (*sensCfgCounter)++;
            sensThreshold=sensThresholds[*sensCfgCounter];
          }
      }
  }
#endif

#ifdef BNO
  void init_BNO(void) {
    while(!bno085.begin_I2C()) {
      #ifdef DEB
      Serial.println("Failed to connect to BNO085");
      #endif
      delay(500);
    }
    #ifdef DEB
    Serial.println("Connencted. Products ids:");
    for(int n=0; n < bno085.prodIds.numEntries; n++) {
      Serial.print("Part ");
      Serial.print(bno085.prodIds.entry[n].swPartNumber);
      Serial.print(": Version :");
      Serial.print(bno085.prodIds.entry[n].swVersionMajor);
      Serial.print(".");
      Serial.print(bno085.prodIds.entry[n].swVersionMinor);
      Serial.print(".");
      Serial.print(bno085.prodIds.entry[n].swVersionPatch);
      Serial.print(" Build ");
      Serial.println(bno085.prodIds.entry[n].swBuildNumber);
    }
    #endif
    enableReports();
  }
  /**
   * Tell what to capture
   */
  void enableReports() {
    bno085.enableReport(SH2_ROTATION_VECTOR);
    // bno085.enableReport(SH2_GYROSCOPE_CALIBRATED);
  }
  void readRotation(sh2_RotationVectorWAcc &rot) {
    #ifdef DEB
      Serial.print("Rotation - real: ");
      Serial.print(rot.real);
      Serial.print(" i: ");
      Serial.print(rot.i);
      Serial.print(" j: ");
      Serial.print(rot.j);
      Serial.print(" k: ");
      Serial.println(rot.k);
    #endif
    //TODO: use IMU data
  }
#endif
