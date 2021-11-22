#define DEB

#include <ESP32Servo.h>

//IMU
#define SAMPLERATE_DELAY_MS 500 //how often to read data from the board [milliseconds]

//#define UTR1 //comment if ultrasonic sensor 1 is not present
#define trigPin1  33
#define echoPin1  32
//#define UTR2 //comment if ultrasonic sensor 2 is not present
#define trigPin2  25
#define echoPin2  26
//#define UTR3 //comment if ultrasonic sensor 3 is not present
#define trigPin3  27
#define echoPin3  14

#define SERVO //comment if servo is not present
#define servoPin 12

#define TARGET 80 //target height in mm

//Still to be determined
#define Ki 0.3
#define Kp 0.3
#define Kd 0.3

void triggerMeasure(int8_t trig_pin, int8_t echo_pin, void (*start_count_procedure)());

volatile int64_t startT1,startT2,startT3;
volatile float distance1=0,distance2=0,distance3=0;

esp_timer_handle_t timer;
Servo srv;

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
  
    const esp_timer_create_args_t oneshot_timer_args = {
      .callback = &oneshot_timer_callback,
      /* argument specified here will be passed to timer callback function */
      .arg = (void*) timer
    };
  
    esp_timer_create(&oneshot_timer_args,&timer);  
    esp_timer_start_once(timer, 5000000000);
    
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
}
    
void loop() {
    static uint32_t timer=0;  
    static int8_t rotationAngle;
    static int32_t measuredHeight,error,prevError=0;
    static float cumulativeError,rateError;
    if (millis() - timer > SAMPLERATE_DELAY_MS) {
      timer = millis(); // reset the timer

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
      error = TARGET - measuredHeight;
      cumulativeError = error * (SAMPLERATE_DELAY_MS / 1000); //integral of the error, SAMPLERATE is the elapsed time
      rateError = (error - prevError)/(SAMPLERATE_DELAY_MS / 1000); //derivative of the error      
      rotationAngle = (int8_t)(Kp * error + Ki * cumulativeError + Kd * rateError);
      #ifdef SERVO        
        srv.write(rotationAngle);
      #endif
      prevError = error;
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

void startCount(volatile int64_t *start_time,int8_t echo_pin,void (*stop_count_procedure)()){
    *start_time = esp_timer_get_time();
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
    double durationMicros = esp_timer_get_time()-start_time;
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

static void oneshot_timer_callback(void* arg){}

void init_servo(){
   srv.attach(servoPin); 
}
