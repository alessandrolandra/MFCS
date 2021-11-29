#include "inc/stm32f411xe.h"

void busy_delay(int ms);

int main(void){
	RCC->AHB1ENR |= 1;
	GPIOA->MODER &= ~0x00000C00;
	GPIOA->MODER |=  0x00000400;
	while(1){
		GPIOA->ODR |= 0x00000020;
		busy_delay(250);
		GPIOA->ODR &= ~0x00000020;
		busy_delay(250);
	}
}

void busy_delay(int ms){
	int i;
	for (; ms >0 ; ms--)
		for(i=0;i<3195;i++);
}