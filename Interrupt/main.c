#include <stdint.h>
#include <string.h>
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

static int second = 0;

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
	
	GPIO_PORTF_IS_R &= ~0x10;        // 8) edge triggered	
	GPIO_PORTF_IBE_R &= ~0x10;			 // 9) interrupt controlled by GPIOIEV
	GPIO_PORTF_IEV_R &= ~0x10;			 // 10) active on falling edge
	GPIO_PORTF_IM_R |= 0x11;         // 11) enable of SW1 and SW2
 
	// enable port F and set priority to( 2 )
	NVIC_EN0_R |= (1<< 30);
	NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00400000;
}


// Initialize the SysTick timer
void SysTick_Init(void) {
    NVIC_ST_CTRL_R = 0;    
	// Disable SysTick during setup
    NVIC_ST_RELOAD_R = 15999999;  // 1sec * 16 MHz - 1
    NVIC_ST_CURRENT_R = 0;          // Clear current value
    NVIC_ST_CTRL_R = 0x00000007;    // Enable SysTick with processor clock and interrupt
		NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x20000000; // Set priority to 1
}

void SysTick_Handler(void){
	second++;
	GPIO_PORTF_DATA_R ^= GREEN; // toggle GREEN LED at each overflow (which happens every 1 sec)
	
}

void GPIOF_Handler(void){
		
		// check if switch 1 is clicked
		if(GPIO_PORTF_MIS_R & SW1){
			GPIO_PORTF_DATA_R = RED;
			NVIC_ST_CTRL_R = 0x00; // pause SysTick
			GPIO_PORTF_ICR_R = SW1; // this interrupt handled
		} 
		
		// check if switch 2 is clicked
		if(GPIO_PORTF_MIS_R & SW2){
			GPIO_PORTF_DATA_R = 0; // turn off all leds
			NVIC_ST_CTRL_R = 0x00000007; // enable SysTick again
			GPIO_PORTF_ICR_R = SW2; // this interrupt handled
		}
	
}  

int main(void){ 
	PortF_Init();
	SysTick_Init();
	__enable_irq();
	while(1){
	}
	
}

