// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"



#define LIGHT                   (*((volatile unsigned long *)0x400050FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050FC)) // bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
	
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x4002400C)) // bits 1-0
//#define SENSOR                  (*((volatile unsigned long *)0x4002400C))
#define SENSOR									(*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control

#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define WALK_LIGHT       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))



// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void Port_Init(void);
void PortF_Init(void);
void delayms(unsigned long ms);

// Linked data structure
struct State {
  unsigned long Out; 
	unsigned long OutP;
  unsigned long Time;  
  unsigned long Next[8];}; 
typedef const struct State STyp;
	#define GoW 0
	#define WaitW 1
	#define GoS 2
	#define WaitS 3
	#define Walk 4
	#define DontWalk 5
	#define DontWalkOn 6
	#define DontWalkOff 7
	#define DontWalkOn2 8
	#define DontWalkOff2 9
	
	STyp FSM[11]={
		{0x0C,0x02,800,{GoW,GoW,WaitW,WaitW,WaitW,WaitW,WaitW,WaitW}},
		{0x14,0x02,500,{GoS,GoS,GoS,GoS,Walk,Walk,GoS,GoS}},
		{0x21,0x02,800,{GoS,WaitS,GoS,WaitS,WaitS,WaitS,WaitS,WaitS}},
		{0x22,0x02,500,{Walk,GoW,Walk,GoW,Walk,Walk,Walk,Walk}},
		{0x24,0x08,800,{Walk,DontWalk,DontWalk,DontWalk,Walk,DontWalk,DontWalk,DontWalk}},
		{0x24,0x02,500,{DontWalkOff,DontWalkOff,DontWalkOff,DontWalkOff,DontWalkOff,DontWalkOff,DontWalkOff,DontWalkOff}},
		{0x24,0x02,100,{DontWalkOff2,DontWalkOff2,DontWalkOff2,DontWalkOff2,DontWalkOff2,DontWalkOff2,DontWalkOff2,DontWalkOff2}},
		{0x24,0,100,{DontWalkOn,DontWalkOn,DontWalkOn,DontWalkOn,DontWalkOn,DontWalkOn,DontWalkOn,DontWalkOn}},
		{0x24,0x02,100,{GoW,GoW,GoS,GoW,GoW,GoW,GoS,GoW}},
		{0x24,0,100,{DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2,DontWalkOn2}}
	};
	unsigned long S;  // index to the current state 
	unsigned long Input; 
// ***** 3. Subroutines Section *****

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	Port_Init();
	S=Walk;
  
  EnableInterrupts();
  while(1){
    LIGHT=FSM[S].Out;
		WALK_LIGHT=FSM[S].OutP;
		delayms(FSM[S].Time);
		Input=SENSOR;
		S=FSM[S].Next[Input];
		
		
  }
}

void Port_Init(void){volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x12;      // 1) B E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x03; // 3) disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE1-0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
	PortF_Init();
}

void PortF_Init(void){ volatile unsigned long delayy;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delayy = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF2,PF1  
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x00;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}



void delayms(unsigned long ms){
  unsigned long count;
  while(ms > 0 ) { // repeat while there are still ms to delay
    count = 16000; // number of counts to delay 1ms at 80MHz
    while (count > 0) { 
      count--;
    } // This while loop takes approximately 3 cycles
    ms--;
  }
}
