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

void UART1_Init(void){
	SYSCTL_RCGCUART_R |= 0x02;
	SYSCTL_RCGCGPIO_R |= 0x02;
	
	UART1_CTL_R &= ~0x01; //disable uart
	UART1_IBRD_R = 104 ; //integer baud rate divisor
	UART1_FBRD_R = 11 ; //float baud rate divisor
	UART1_LCRH_R |= 0x70 ; //fifo enable and width  8 bits
	UART1_CTL_R = 0X0301 ; //uart enable , rx enable , tx enable
	GPIO_PORTB_DEN_R |= 0X03; //digital enable

  GPIO_PORTB_AMSEL_R &= ~0x03; //clear analog mode
	GPIO_PORTB_AFSEL_R |= 0x03; //set alternate function 
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R &= ~0xFF) | 0X00000011; //clear PB0 and PB1 then set them as uart
	
}

uint8_t UART1_read(void){
	
	while ((UART1_FR_R & 0x0010) != 0);
	return (char)(UART1_DR_R & 0xFF);
}

void UART1_write_char(char c){
	
	while ((UART1_FR_R & 0x0020) != 0){};
	UART1_DR_R = c;
	
}

void UART1_write_string (char *ptr){
	
	while(*ptr){
		UART1_write_char(*ptr);
		ptr++;		
	}
}	

void getCommand(char *Command, int len){
	
	char character;
	int i;
	for(i = 0; i < len; i++){
		
		character = UART1_read();
		
	  if(character == '\r')
			break;
		
		Command[i] = character;
		UART1_write_char(Command[i]);
	}
}

const int len = 120;
char Command[len] = {0};

int main(void){ 
	
	UART1_Init();
	PortF_Init();             
	SysTick_Init();   

	while(1){
		PortF_Output(0x02);
		UART1_write_string("Enter Command:\n");
		getCommand(Command, len);
		
		if(strcmp(Command, "A") == 0){
			delay(60);
			PortF_Output(RED);
		}
		else if(strcmp(Command, "B") == 0){
			delay(30);
			PortF_Output(BLUE);
		}
		else if (strcmp(Command, "D") == 0){
			delay(120);
			PortF_Output(GREEN);
		}
		
		memset(Command, 0, len);
			
	}
}