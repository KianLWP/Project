//====================================================================
// INCLUDE FILES
//====================================================================
#include "stm32f0xx.h"
#include "stdint.h"
#include <stdio.h>
//====================================================================
// SYMBOLIC CONSTANTS
//====================================================================
#define T_delay 66500	//*10uS
#define M_delay 500
#define S_delay 10000
#define endTimeMax 500 //in ms
#define endTimeMin 300
#define juncTime 1000 //in ms, used to differentiate between junction and end
#define outer 60	//outer wheel speed
#define inner 60	//inner wheel speed
#define rightSpeed 68
#define leftSpeed 70
#define speed 75
#define straight 0b0110
#define left 0b1010
#define right 0b0101
//====================================================================
// GLOBAL VARIABLES
//====================================================================
int sensLMid;
int sensRMid;
int sensRight;
int sensLeft;
int myTicks;
int Lights=0;
int timer=0; //0 if timer is currently off
int time=0;
int turning=0; //0 if not turning, 1=left 2=right 3=u-turn
int adjustL=0;
int adjustR=0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
//Init
void init_PWM(void);
void init_TIM14(void);
void init_Sensors(void);
void init_Outputs(void);
void init_LEDs(void);
void init_StartBtn(void);

//Tim and Interrupts
void TIM14_IRQHandler(void);
void EXTI4_15_IRQHandler(void);

//Movement commands
void start(void);
void forward(void);
void turnRight(void);
void turnLeft(void);
void turn(int LeftRight);	//1=left 2=right 3=u-turn

//Timer functions
void Delay(int x); //Delay in 10uS
void startTimer(void); //Timer
void stopTimer(void); //Timer
void stop(void); //stop robot

//
void RunLeft(void);	//Always Left
void RunRight(void); //Always Right
void adjust(int y); //1 to the left 2 to the right
//====================================================================
// MAIN FUNCTION
//====================================================================
void main(void)
{
																																																																																																																																																																																																	init_Outputs();
	init_TIM14();
	init_PWM();
	//init_LEDs();
	init_Sensors();
	init_StartBtn();
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable EXTI4-7 and 14 interrupt in the NVIC


	start();
	for (;;){
		//Delay(5); //"Debounce" time
		if (turning==0) {
			if ((time < endTimeMax) && (time > endTimeMin)) {
				//end detected
			} else {
				if (sensLeft==1) {
					//stop();
					turn(1);
				} else if ((sensRMid==1) && (sensLMid==1)) {
					forward();
				} else if (sensRight==1) {
					//stop();
					turn(2);
				} else if ((sensRMid==0) && (sensLMid==0)) {
					//stop();
					turn(3);
				} else {
					if ((sensLMid==1) && (sensRMid==0)) { //Right off so needs to go left
						adjust(1);
					}
					if ((sensLMid==0) && (sensRMid==1)) { //Left off so needs to go right
						adjust(2);
					}
					if ((sensLMid==1) && (sensRMid==1) && (adjustR||adjustL)) { //Both on so speed reset to default
						forward();
					}
				}

			}
		} else if (turning==1) {
			if (sensLMid==1) {
				//stop();
				forward();
			}
		} else if (turning==2) {
			if (sensRMid==1) {
				//stop();
				forward();
			}
		} else if (turning==3) { //different if statement just in case we build onto the algorithm
			if (sensLMid==1) {
				//stop();
				forward();
			}
		}

	}
}

//====================================================================
// INITIALIZE FUNCTIONS
//====================================================================
void init_PWM(){ //This section initialises the PWM to pin PB10 and PB11
	//PWM pins
    GPIOB->MODER |=  (GPIO_MODER_MODER10_1); //Enable pwm motor 1, alternate function
    GPIOB->MODER |=  (GPIO_MODER_MODER11_1); //Enable pwm motor 2

    //PWM Coding
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    //map pins 10 and 11 to the individual channels within timer2
    GPIOB->AFR[1] |= (2 << (4*(10-8)));
    GPIOB->AFR[1] |= (2 << (4*(11-8)));

    //define initial frequency
    TIM2->ARR = 8000; //1kHz

    //specify pwm mode
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
    TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);

    //capture compare enable register
    TIM2-> CCER |= TIM_CCER_CC3E;
    TIM2-> CCER |= TIM_CCER_CC4E;

    //enable the counter for timer 2
    TIM2->CR1 |= TIM_CR1_CEN;

    TIM2->CCR3 = rightSpeed * 80;
    TIM2->CCR4 = leftSpeed * 80;
}

void init_LEDs(void) {
	GPIOB->MODER |= 0b101010100000000; //Enable LEDs, PB4-7
	GPIOB->ODR &= ~0b1111111;
}

void init_StartBtn(void)
{
	GPIOA->MODER &= ~GPIO_MODER_MODER8; //set PB14 to input
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0;

    SYSCFG -> EXTICR[2] &= SYSCFG_EXTICR3_EXTI8_PA;
	EXTI -> IMR |= EXTI_IMR_MR8; // unmask external interrupt 0-3
	EXTI -> RTSR &= ~EXTI_RTSR_TR8;
	EXTI -> FTSR |= EXTI_FTSR_TR8;
}
void init_TIM14(void)
{
	// set up a 0.1 s interrupt.
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; // clock to the timer
	TIM14->PSC = 0;
	TIM14->ARR = 7999; // 1Khz
	TIM14->CR1 |= TIM_CR1_URS; // only overflow/underflow generates update
	TIM14->DIER |= TIM_DIER_UIE; // enable the update event interrupt
	TIM14->EGR |= TIM_EGR_UG;
	TIM14->CR1 |= TIM_CR1_CEN;
	// enable interrupt in NVIC
	NVIC_EnableIRQ(TIM14_IRQn);

}

void init_Sensors (void)
{
	GPIOA->MODER &= ~GPIO_MODER_MODER4;	//Digital input for sensors
	GPIOA->MODER &= ~GPIO_MODER_MODER5;
	GPIOA->MODER &= ~GPIO_MODER_MODER6;
	GPIOA->MODER &= ~GPIO_MODER_MODER7;

	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR5_1|GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_1);
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // enable clock for the sys

	SYSCFG -> EXTICR[1] &= 0b0; // map EXTI4-7 to PA
	EXTI -> IMR |= 0b11110000; // unmask external interrupt 4-7
	EXTI -> FTSR |= 0b11110000; // trigger on falling edge, front sensors. pins 5/6
	EXTI -> RTSR |= 0b11110000; //trigger on rising edge, side sensors. pins 4/7
}

void init_Outputs(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //initialize Clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//General Pins
	//GPIOA->MODER &= 0;  not necessary to set PA4-7 to digital inputs since default is 0; 00 = Digital Input Mode (reset state)
	GPIOA->MODER |= (0b01010101); //Digital outputs to IC1-4
	GPIOB->MODER |= GPIO_MODER_MODER7_0; //Digital Output for Red Led, PB7
}

//====================================================================
// INTERRUPT HANDLERS
//====================================================================
void EXTI4_15_IRQHandler(void)
{
	//GPIOA->ODR &= ~0b1111;
	if ((EXTI->PR & EXTI_PR_PR8)==EXTI_PR_PR8){
		EXTI->PR |= EXTI_PR_PR8; //Clear the interrupt pending bit

		start();
	}
	if ((EXTI->PR & EXTI_PR_PR4)==EXTI_PR_PR4){ //Checks if EXTI4 was called
		EXTI->PR |= EXTI_PR_PR4; //Clear the interrupt pending bit
	    if (turning==0) {	//Used to differentiate between end or junction
			if ((GPIOA->IDR & GPIO_IDR_4)==GPIO_IDR_4){

				timer=1;
				sensLeft=0;
			} else {
				time=myTicks;
				timer=0;
				sensLeft=1;
			}
	    } else {
	    	if ((GPIOA->IDR & GPIO_IDR_4)==GPIO_IDR_4){ //used to detect current sensor state
	    		sensLeft=1;
	    	} else {
	    		sensLeft=0;
	    	}
	    }
	}
	if ((EXTI->PR & EXTI_PR_PR5)==EXTI_PR_PR5){ //Checks if EXTI5 was called
		EXTI->PR |= EXTI_PR_PR5; //Clear the interrupt pending bit
		if ((GPIOA->IDR & GPIO_IDR_5)==GPIO_IDR_5){ //used to detect current sensor state
			sensLMid=1;
		} else {
			sensLMid=0;
		}

	}
	if ((EXTI->PR & EXTI_PR_PR6)==EXTI_PR_PR6){ //Checks if EXTI62 was called
		EXTI->PR |= EXTI_PR_PR6; //Clear the interrupt pending bit
		if ((GPIOA->IDR & GPIO_IDR_6)==GPIO_IDR_6){ //used to detect current sensor state
			sensRMid=1;
		} else {
			sensRMid=0;
		}
	}
	if ((EXTI->PR & EXTI_PR_PR7)==EXTI_PR_PR7){ //Checks if EXTI7 was called
		EXTI->PR |= EXTI_PR_PR7; //Clear the interrupt pending bit
		if ((turning==0) && (timer==0)) {	//Prevents turning before the right sensor senses white again, only used when there isn't a path on the left which will trigger the timer
			if ((GPIOA->IDR & GPIO_IDR_7)== 0b0){
				sensRight=1;
			}
		} else {
			if ((GPIOA->IDR & GPIO_IDR_7)==GPIO_IDR_7){ //used to detect current sensor state
				sensRight=1;
			} else {
				sensRight=0;
			}
		}

	}

}

void TIM14_IRQHandler(void)
{
	myTicks++;
	// ack interrupt
	TIM14->SR &= ~TIM_SR_UIF;
}
//====================================================================
// TIMER FUNCTIONS
//====================================================================
void Delay(int x)
{
	TIM14->ARR = 79; // 100Khz
	myTicks=0;
	TIM14->CR1 |= TIM_CR1_CEN;
	while (myTicks<x);
	TIM14->CR1 &= ~TIM_CR1_CEN;
}

void startTimer(void)
{
	TIM14->ARR = 7999; // 1Khz
	myTicks=0;
	TIM14->CR1 |= TIM_CR1_CEN;
}

void stopTimer(void)
{
	TIM14->CR1 &= ~TIM_CR1_CEN;
}
//====================================================================
// MOVE FUNCTIONS
//====================================================================

void forward(void)
{
	turning = 0;
	sensLeft=0;
	sensRight=0;
	GPIOA->ODR &= ~0b1111;
	GPIOA->ODR |= straight;
	TIM2->CCR3 = rightSpeed * 80;
	TIM2->CCR4 = leftSpeed * 80;
}

void turnRight(void)
{
	GPIOA->ODR &= ~0b1111;
	GPIOA->ODR |= right;
/*	TIM2->CCR3 = speed * 80;
	TIM2->CCR4 = speed * 80;
	Delay(M_delay);*/
    TIM2->CCR3 = inner * 80;	//left speed
    TIM2->CCR4 = outer * 80;	//right speed
}

void turnLeft(void)
{
	GPIOA->ODR &= ~0b1111;
	GPIOA->ODR |= left;
/*	TIM2->CCR3 = speed * 80;
	TIM2->CCR4 = speed * 80;
	Delay(M_delay);*/
	TIM2->CCR3 = outer * 80;
	TIM2->CCR4 = inner * 80;
}
void start(void)
{
	turning=0;
	timer=0;
	forward();
	myTicks=0;

	//init sens variables
	sensLMid=1;
	sensRMid=1;
	sensRight=0;
	sensLeft=0;
}

void stop(void)
{
	GPIOA->ODR &=~0b1111;
	for (int l=0; l<600; l++) {
		for(int i=0; i<3000; i++) {}
	}
}

void adjust(int y)
{
	if (y==2) {
		adjustR=1;
		TIM2->CCR3 = (rightSpeed-20)*80;
	}
	if (y==1) {
		adjustL=1;
		TIM2->CCR4 = (leftSpeed-20)*80;
	}
}

void turn(int LeftRight)
{
	GPIOB->ODR |= GPIO_ODR_7;
	GPIOA->ODR &=~0b1111;
	GPIOA->ODR |= straight;
	for (int l=0; l<350; l++) {
		for(int i=0; i<600; i++) {}
	}
	GPIOB->ODR &= ~GPIO_ODR_7;
	sensLMid=0;
	sensRMid=0;
	turning = LeftRight;
	if (LeftRight==1){	//left
		turnLeft();
	} else if(LeftRight==2)	{
		turnRight();
	} else {
		turnLeft();
	}

}
