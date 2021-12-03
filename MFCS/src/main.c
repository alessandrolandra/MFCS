#include <stm32f411xe.h>//oppure: #include "stm32f4xx.h"
#include "arm_math.h"
#include "MFRC522.h"
#include <stdio.h> // just for test purposes

//target height in mm
#define TARGET1 1000
#define TARGET2 850
#define TARGET3 700

//sensibility threshold in mm (how much distance from the target to change PID parameters)
#define SENS1 50
#define SENS2 75
#define SENS3 100

#define AGG_KP 4
#define AGG_KI 0.2
#define AGG_KD 1

#define CONS_KP 1
#define CONS_KI 0.05
#define CONS_KD 0.25

/* Uncomment to set custom pulse length for 0° rotation */
//#define SERVO_MICROS_MIN    1000
 
/* Uncomment to set custom pulse length for 180° rotation */
//#define SERVO_MICROS_MAX    2000

__IO uint32_t msTick;// tick counter

const char Mx1[7][5]={{0x12,0x45,0xF2,0xA8},{0xB2,0x6C,0x39,0x83},{0x55,0xE5,0xDA,0x18},{0x1F,0x09,0xCA,0x75},{0x99,0xA2,0x50,0xEC},{0x2C,0x88,0x7F,0x3D}}; //card identifier 

int abs(int x);
void updateAngle(arm_pid_instance_q31* PID,int8_t target,int8_t sens_threshold);
int8_t readRFID();

int main(){
  static arm_pid_instance_q31 PID;
  static int16_t target=TARGET1;
  static int8_t sens_threshold=SENS1;
  //static TM_SERVO_t srv;

  PID.Kp = AGG_KP;
  PID.Ki = AGG_KI;
  PID.Kd = AGG_KD;

  /* Initialize PID system, integer format */
  arm_pid_init_q31(&PID, 1);

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

  //Set PWM duty cycle for DC FAN to cool down sensor for "TEMP_CURRENT"
  TM_PWM_SetChannelPercent(&TIM_Data, TM_PWM_Channel_1, duty);
  */

  SystemInit();

  //TM_DELAY_Init();

  //TM_SERVO_Init(&srv, TIM2, TM_PWM_Channel_1, TM_PWM_PinsPack_2);

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
  #endif*/

  // Update SystemCoreClock variable
  SystemCoreClockUpdate();
  // Make SysTick overflow every 1ms
  // The SysTick clock source is 168 MHz so 168,000,000 ticks per second. The time required to make one tick is 1 ÷ 168,000,000 ≈ 5.952 ns
  // 1,000,000 ÷ 5.952 ≈ 168,010.752 ticks. But, the data type of input parameter of that function is uint32_t so: 168,000. SystemCoreClock is a global variable that contains the system frequency which is 168MHz
  SysTick_Config(SystemCoreClock / 1000);

  MFRC522_Init();

  while (1) {
    updateAngle(&PID,target,sens_threshold);
    // Set delay to 500ms
    msTick = 500;
    // Do nothing until msTick is zero
    while (msTick);
  } 
}

void updateAngle(arm_pid_instance_q31* PID,int8_t target,int8_t sens_threshold){
  int16_t measuredHeight,gap;
  int8_t rotationAngle;

  //measuredHeight = (int)(((distance2+distance3)/2)+distance1)/2;
  measuredHeight = 0;
  gap = abs(measuredHeight - target);
  if (gap < sens_threshold){//close to target, use conservative tuning parameters
    PID->Kp = CONS_KP;
    PID->Ki = CONS_KI;
    PID->Kd = CONS_KD;
  }else{//far from target, use aggressive tuning parameters
    PID->Kp = AGG_KP;
    PID->Ki = AGG_KI;
    PID->Kd = AGG_KD;
  }
          
  /* Calculate PID here, argument is error */
  /* Output data will be returned, we will use it as duty cycle parameter */
  rotationAngle = arm_pid_q31(PID, gap);           
  
  //TM_SERVO_SetDegrees(&srv, rotationAngle);
}

// This function will be called every 1ms
void SysTick_Handler(void){
  msTick--;
}

int abs(int x){
  return x>=0?x:-x;
}

int8_t readRFID(){
  char status,str[90],cardstr[MAX_LEN+1],UID[5],SectorKey[7];
  status = MFRC522_Request(PICC_REQIDL, cardstr);
	if(status == MI_OK) {
	  sprintf(str,"Card:%x,%x,%x", cardstr[0], cardstr[1], cardstr[2]);
	  // Anti-collision, return card serial number == 4 bytes
	  status = MFRC522_Anticoll(cardstr);
	  if(status == MI_OK) {
		  sprintf(str,"UID:%x %x %x %x", cardstr[0], cardstr[1], cardstr[2], cardstr[3]);
		  UID[0] = cardstr[0];
		  UID[1] = cardstr[1];
		  UID[2] = cardstr[2];
		  UID[3] = cardstr[3];
		  UID[4] = cardstr[4];
		  status = MFRC522_SelectTag(cardstr);
		  if (status > 0){
			  SectorKey[0] = ((Mx1[0][0])^(UID[0])) + ((Mx1[0][1])^(UID[1])) + ((Mx1[0][2])^(UID[2])) + ((Mx1[0][3])^(UID[3]));// 0x11; //KeyA[0]
			  SectorKey[1] = ((Mx1[1][0])^(UID[0])) + ((Mx1[1][1])^(UID[1])) + ((Mx1[1][2])^(UID[2])) + ((Mx1[1][3])^(UID[3]));// 0x11; //KeyA[0]
			  SectorKey[2] = ((Mx1[2][0])^(UID[0])) + ((Mx1[2][1])^(UID[1])) + ((Mx1[2][2])^(UID[2])) + ((Mx1[2][3])^(UID[3]));// 0x11; //KeyA[0]
			  SectorKey[3] = ((Mx1[3][0])^(UID[0])) + ((Mx1[3][1])^(UID[1])) + ((Mx1[3][2])^(UID[2])) + ((Mx1[3][3])^(UID[3]));// 0x11; //KeyA[0]
			  SectorKey[4] = ((Mx1[4][0])^(UID[0])) + ((Mx1[4][1])^(UID[1])) + ((Mx1[4][2])^(UID[2])) + ((Mx1[4][3])^(UID[3]));// 0x11; //KeyA[0]
			  SectorKey[5] = ((Mx1[5][0])^(UID[0])) + ((Mx1[5][1])^(UID[1])) + ((Mx1[5][2])^(UID[2])) + ((Mx1[5][3])^(UID[3]));// 0x11; //KeyA[0]
			  status = MFRC522_Auth(0x60, 3, SectorKey, cardstr);
			  if (status == MI_OK){
				  sprintf(str, "Auth. OK");
			  }else{
				  //another sector key
			  }
			  MFRC522_StopCrypto1();
		  }
	  }
	  MFRC522_Halt();
	}else{
	  //Waiting for Card
	}
}