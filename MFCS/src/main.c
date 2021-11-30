#include <stm32f411xe.h>
#include <Ultrasonic.h>

#define TARGET 800 //target height in mm

#define AGG_KP        4
#define AGG_KI        0.2
#define AGG_KD        1

#define CONS_KP        1
#define CONS_KI        0.05
#define CONS_KD        0.25

arm_pid_instance_f32 PID;

int main(){

  PID.Kp = AGG_KP;
  PID.Ki = AGG_KI;
  PID.Kd = AGG_KD;

  /* Initialize PID system, float32_t format */
  arm_pid_init_f32(&PID, 1);

  /*myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 20);//10-13 degrees down, 8-9 degrees up*/

/*
  //Timer data for PWM
  TM_PWM_TIM_t TIM_Data;
  //Initialize TIM2, 1kHz frequency
  TM_PWM_InitTimer(TIM2, &TIM_Data, 1000);    
  //Initialize TIM2, Channel 1, PinsPack 2 = PA5
  TM_PWM_InitChannel(&TIM_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2);    
  //Set default duty cycle
  TM_PWM_SetChannelPercent(&TIM_Data, TM_PWM_Channel_1, duty);
*/

  SystemInit();

  /*#ifdef UTR1
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
  */

  while (1) {
    
  } 
}

void updateAngle(){
  float measuredHeight,gap;
  int rotationAngle;

  measuredHeight = (((distance2+distance3)/2)+distance1)/2;
  /* Calculate error */
  gap = abs(measuredHeight - TARGET);
  if (gap < 50){//close to target, use conservative tuning parameters
    PID.Kp = CONS_KP;
    PID.Ki = CONS_KI;
    PID.Kd = CONS_KD;
  }else{//far from target, use aggressive tuning parameters
    PID.Kp = AGG_KP;
    PID.Ki = AGG_KI;
    PID.Kd = AGG_KD;
  }
          
  /* Calculate PID here, argument is error */
  /* Output data will be returned, we will use it as duty cycle parameter */
  rotationAngle = arm_pid_f32(&PID, gap);
           
  /* Set PWM duty cycle for DC FAN to cool down sensor for "TEMP_CURRENT" */
  //TM_PWM_SetChannelPercent(&TIM_Data, TM_PWM_Channel_1, duty);
}