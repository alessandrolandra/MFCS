/** @Brief: Test delayMicroseconds() procedure
 *  @Info:  Test precision on stm32f4 nucleo board with 100MHz core
 *  @Note:  Code can be adapted to Arduino Uno board by commenting board_directive
 */

/**
 * @defgroup board_directive
 * @{
 */
 /** comment this if the board is Arduino Uno **/
#define STM32_B
/** @} */

/**
 * @defgroup pinout_arduino1_stm32f411re
 * @{
 */
#ifdef STM32_B
/** ST morpho pinout **/
  #define trigPin1 PA5
  #define echoPin1 PB3
  #define trigPin2 PB5
  #define echoPin2 PA10  
  #define trigPin3 PA9
  #define echoPin3 PA8
#else
/** Arduino Uno digital pins pinout **/
  #define trigPin  13
  #define echoPin1 3
  #define trigPin  4
  #define echoPin1 2
  #define trigPin3 8
  #define echoPin3 7
#endif
/** @} */

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP");
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  Serial.println("SETUP DONE");
  /** test trigger mesure 1, 20 us test **/
  digitalWrite(trigPin1, HIGH);
}

void triggerMeasure(int8_t trig_pin, int8_t delta){
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(delta);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(delta);
}

void loop() {
  triggerMeasure(trigPin1,1);
}
