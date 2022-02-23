#define DEB
//#define ESP32 //uncomment to use ESP32
/**
 * UTR 1 is the ultrasonic sensor positioned at the bow (prua)
 * UTR 2 is at stern (poppa), to the LEFT
 * UTR 3 is at stern (poppa), to the RIGHT
 * Below the real mounting distances from the BNO and the offsets to be subtracted from UTRs to consider every sensor on the same plain are reported (both in mm)
 */

#define UTR1_BNO_DISTANCE 515
#define UTR2_BNO_DISTANCE_X 155
#define UTR2_BNO_DISTANCE_Y 175
#define UTR3_BNO_DISTANCE_X 155
#define UTR3_BNO_DISTANCE_Y 175

#define UTR1_OFFSET_HEIGHT 51
#define UTR2_OFFSET_HEIGHT 52
#define UTR3_OFFSET_HEIGHT 52
 
#define UTR1 //comment if ultrasonic sensor 1 is not present
#define UTR2 //comment if ultrasonic sensor 2 is not present
#define UTR3 //comment if ultrasonic sensor 3 is not present
#define SERVO //comment if servo is not present
#define MFRC //comment if MFRC is not present
#define BNO //comment if BNO is not present
#define WATCHDOG  //comment if WATCHDOG is not present
#define BUZZ //comment if buzzer is not present
//#define WATCHDOG_TEST //uncomment to TEST the watchdog (WATCHDOG define needed)

#include <PID_v1.h>
#ifndef ESP32
  #include <Servo.h>
#endif
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_BNO08x.h>
#include <IWatchdog.h>

//#define SAMPLERATE_DELAY_MS 500 //how often to read data from the board [milliseconds]
#define SAMPLERATE_DELAY_MS 250 //how often to read data from the board [milliseconds]

#ifdef BUZZ  
  #define NOTE_C5  523
  #define NOTE_D5  587
  #define NOTE_E5  659
  #define NOTE_F5  698
  #define NOTE_G5  784
  #define NOTE_A5  880
  #define NOTE_B5  988
  #define NOTE_C6  1047
#endif

#ifdef ESP32
  #define trigPin1  12
  #define echoPin1  13
  #define trigPin2  14
  #define echoPin2  27
  #define trigPin3  26
  #define echoPin3  25
  #define servoPin  12
  #define MFRC_SS   10
  #define MFRC_RST  9
  #define BUZZER_PIN 28
#else
  #define trigPin1  PA0
  #define echoPin1  PB3
  #define trigPin2  PA1
  #define echoPin2  PB4
  #define trigPin3  PA4
  #define echoPin3  PB5
  #define servoPin  PB10
  #define MFRC_SS   PA8 //D7
  #define MFRC_RST  PA9 //D8
  #define BUZZER_PIN PC7
#endif

/*
 * MFRC ESP32 pinout
 * SS  10
 * RST 9
 * SDA 21  
 * SCL 22
 * 
 * MFRC STM32 pinout
 * SS  PA8
 * RST PA9
*/
#ifdef MFRC
  MFRC522 mfrc522(MFRC_SS, MFRC_RST);
#endif

//aggressive PID parameters
double aggKi=0,aggKp=0.7,aggKd=0.05;
//conservative PID parameters
double consKi=0,consKp=0.5,consKd=0.01;

double targetHeights[3]={700,850,1000};//target heights in mm
double sensThresholds[3]={50,75,100};//sensibility thresholds in mm (how much distance from the target to change PID parameters)
double targetHeight=targetHeights[1];
uint16_t sensThreshold=sensThresholds[1];

double measuredHeight, rotationDutycycle;//PID input and output

volatile unsigned long startT1,startT2,startT3;
volatile float distance1=0,distance2=0,distance3=0;

const byte heightId[4] = {9,133,241,194};  //TARGET HEIGHT TAG
const byte sensId[4] = {20,110,103,43};  //TARGET SENS TAG

unsigned int soundVect[] = {NOTE_C5, NOTE_G5, NOTE_C6, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5};
unsigned int soundDuration = 125;  //note duration in milliseconds

#ifdef SERVO
  Servo srv;
#endif

/*
 * BNO ESP32 pinout
 * SDA 21  
 * SCL 22
 * 
 * BNO STM32 pinout
 * SDA PB9  
 * SCL PB8
*/
#ifdef BNO
  Adafruit_BNO08x bno085(-1); // reset pin not required in I2C
#endif

PID myPID(&measuredHeight, &rotationDutycycle, &targetHeight, aggKp, aggKi, aggKd, DIRECT);

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

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
    myPID.SetSampleTime(SAMPLERATE_DELAY_MS);
    myPID.SetOutputLimits(1000, 1380); //constrain to usable crank range

    #ifdef WATCHDOG
      IWatchdog.begin(5000000); // Initialize the IWDG with 5 seconds timeout.
      //When the timer reaches zero the hardware block would generate a reset signal for the CPU
    #endif

    #ifdef BUZZ
      pinMode(BUZZER_PIN, OUTPUT);      
    #endif

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
    static double error;
    static uint8_t sCnt1=0;
    static sh2_SensorValue_t sensorValue;
	  uint32_t wdTimeout;
    static int8_t firstCalibration=1;
    
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
        if(checkCalibration(&sensorValue)==0){
          if(firstCalibration){
            #ifdef BUZZ
              tone(BUZZER_PIN, NOTE_C5, soundDuration);
            #endif
            #ifdef DEB
              Serial.println("CALIBRATION DONE");
            #endif
            firstCalibration = !firstCalibration;
          }
          if(sensorValue.sensorId == SH2_ARVR_STABILIZED_RV){
            quaternionToEuler(sensorValue.un.arvrStabilizedRV, &ypr);
            measuredHeight = measureHeight(ypr,distance1,distance2,distance3);
          }
        }else{   
          #ifdef DEB
            Serial.println("IMU Low accuracy, just UTRs considered");
          #endif
          measuredHeight = (((distance2+distance3)/2)+distance1)/2;
        }
      #else
        measuredHeight = (((distance2+distance3)/2)+distance1)/2;
      #endif
      #ifdef DEB
        Serial.print("measured height: ");
        Serial.println(measuredHeight);
      #endif
      
      error = abs(targetHeight-measuredHeight);//distance away from target
      if (error < sensThreshold){//close to target, use conservative tuning parameters
          myPID.SetTunings(consKp, consKi, consKd);
      }else{//far from target, use aggressive tuning parameters
          myPID.SetTunings(aggKp, aggKi, aggKd);
      }
      myPID.Compute();
      #ifdef SERVO
          #ifdef DEB
            Serial.print("rotation dutycycle ");
            Serial.println(rotationDutycycle);
          #endif
        srv.writeMicroseconds(rotationDutycycle);
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

      #ifdef WATCHDOG
        if (IWatchdog.isReset()){  //returns if the system has resumed from IWDG reset.
            #ifdef DEB
              Serial.println("System has resumed due to watchdog elapsed");
            #endif
            IWatchdog.clearReset(); 
          }
        IWatchdog.reload();  // reloads the counter value every time

        #ifdef WATCHDOG_TEST
          Serial.println("DELAY 30s");
          delay(30000);//simulate program stop
        #endif
      #endif
      
      #ifdef BUZZ
    	  if(heightCfgCounter>0 && resetFlagHeight == 0 && noCard%2!= 0){
      	  tone(BUZZER_PIN, soundVect[heightCfgCounter-1], soundDuration);
    	  }
    	  if(sensCfgCounter>0 && resetFlagSense == 0 && noCard%2!= 0){
      	  tone(BUZZER_PIN, soundVect[sensCfgCounter+3], soundDuration);
    	  }    
      #endif
    }
}

int checkCalibration(sh2_SensorValue_t *sensorValue){
  if(!bno085.getSensorEvent(sensorValue)) {
      #ifdef DEB
        Serial.println("Cannot get sensor event");
      #endif
      return 1;
   }
   if((*sensorValue).status < 2) {
      return 0;
   }
   return 1;
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

void stopCount(volatile int64_t start_time,volatile float *distance,int8_t echo_pin,float offset_height){
    unsigned long durationMicros = micros()-start_time;
    //the speed of sound, in air, at 20 degrees C is 343m/s... half distance in mm: micros/10^6*343*10^3 /2 = micros*0.1715
    *distance = durationMicros*0.1715 - offset_height;
    detachInterrupt(echo_pin);    
}
void stopCount1(){
    stopCount(startT1,&distance1,echoPin1,UTR1_OFFSET_HEIGHT);
}
void stopCount2(){
    stopCount(startT2,&distance2,echoPin2,UTR2_OFFSET_HEIGHT);
}
void stopCount3(){
    stopCount(startT3,&distance3,echoPin3,UTR3_OFFSET_HEIGHT);
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
    bno085.enableReport(SH2_ARVR_STABILIZED_RV);
  }
  float measureHeight(euler_t ypr,float d1,float d2,float d3) {
    float pitchOffsetBow,pitchOffsetStern2,pitchOffsetStern3,rollOffset2,rollOffset3,dd1,dd2,dd3;
    #ifdef DEB
      Serial.print("pitch: ");
      Serial.print(ypr.pitch);
      Serial.print(" roll: ");
      Serial.println(ypr.roll);
    #endif
    /* roll is positive around x clockwise
       pitch is positive around y clockwise */
    pitchOffsetBow = UTR1_BNO_DISTANCE * tan(ypr.pitch);
    pitchOffsetStern2 = UTR2_BNO_DISTANCE_Y * tan(ypr.pitch);
    pitchOffsetStern3 = UTR3_BNO_DISTANCE_Y * tan(ypr.pitch);
    rollOffset2 = UTR2_BNO_DISTANCE_X * tan(ypr.roll);
    rollOffset3 = UTR3_BNO_DISTANCE_X * tan(ypr.roll);
    dd1 = d1 + pitchOffsetBow;
    dd2 = d2 - pitchOffsetStern2 - rollOffset2;
    dd3 = d3 - pitchOffsetStern3 + rollOffset3;
    return (dd1+dd2+dd3)/3;
  }
  void quaternionToEuler(sh2_RotationVectorWAcc_t rotational_vector, euler_t* ypr) {
    float qr = rotational_vector.real;
    float qi = rotational_vector.i;
    float qj = rotational_vector.j;
    float qk = rotational_vector.k;
    
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  }
#endif
