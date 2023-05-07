#include <stdint.h>
#include "tm4c123gh6pm.h"

#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define YELLOW    0x0A
#define CYAN      0x0C
#define PINK   		0x06
#define WHITE     0x0E

void SystemInit() {}

void PortF_Init(void){ 
	
	volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

uint32_t PortF_Input(void){
	
  return (GPIO_PORTF_DATA_R&0x11);  // read PF4,PF0 inputs
}

void PortF_Output(uint32_t data){ // write PF3-PF1 outputs
	
  GPIO_PORTF_DATA_R = data;
}

// Initialize the SysTick timer
void SysTick_Init(void) {
	
  NVIC_ST_CTRL_R = 0;    
	// Disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;  // Maximum reload value
  NVIC_ST_CURRENT_R = 0;          // Clear current value
  NVIC_ST_CTRL_R = 0x00000005;    // Enable SysTick with processor clock
}

// Wait for a given number of clock cycles (1ms per delay value)
void SysTick_Wait_1ms() {
	
  NVIC_ST_RELOAD_R = 79999;      // Set reload value to [1ms * 80MHz - 1]
  NVIC_ST_CURRENT_R = 0;           // Clear current value
  while((NVIC_ST_CTRL_R&0x00010000)==0) {}  // Wait for count flag
}

// Delay in seconds
void delay(unsigned int sec) {
	
  unsigned int i;
  sec = sec * 1000;
	
  for (i = 0; i < sec; i++) {	
    SysTick_Wait_1ms();
  }
}

void map_count_to_LED (uint32_t count){
	
	switch(count){
		case 1:
		PortF_Output(RED);
		break;
		case 2:
		PortF_Output(BLUE);
		break;
		case 3:
		PortF_Output(PINK);
		break;
		case 4:
		PortF_Output(GREEN);
		break;
		case 5:
		PortF_Output(YELLOW);
		break;
		case 6:
		PortF_Output(CYAN);
		break;
		case 7:
		PortF_Output(WHITE);
		break;
	}
}

int main(void){ 
	
	uint32_t count = 1;
	PortF_Init();             
	SysTick_Init(); 
  
	while(1){
		map_count_to_LED(count++);
		delay(1);
		if(count > 7)
			count = 1;
	}
}