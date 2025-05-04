/**
 * Self Balancing Platform
 * Authors: Michael Sportelli, Keegan Nelson
 *
 * Connections:
 *
 */

#include "stm32l552xx.h"
#include "stdio.h"
#include <stdbool.h>
#include "math.h"
#include "string.h"


// Macros
#define bitclear(word, idx)  (word &= ~(1<<idx)) // clears bit #
#define bitset(word, idx)    (word |= (1<<idx) ) // sets bit #
#define bitflip(word, idx)   (word ^= (1<<idx)) // flips bit #
#define bitcheck(word, idx)  ((word>>idx) & 1) //checks bit number 0 means clear
#define delay_ms(val) for (int i = 0; i < (1600 * val); i++) {} // delays x milliseconds

/* Helper functions */
void init_adc1();
void init_LPUART1();
void pwm_blue_x(uint32_t);
void pwm_green_y(uint32_t);
float pid_x(float);
float pid_y(float);
void get_xy(uint16_t *, uint16_t *);
uint16_t rxLPUART1();
void txLPUART1();
void balance_timer();
//void linePattern(float rx, float ry, int wait_ms, int cycles);


/* Init Functions */
void init_clks();
void init_LPUART1();
void init_adc1();

/*Variables*/
#define ADC_MIN   0      // empirical         (≈ left / bottom edge)
#define ADC_MAX  4096      // empirical         (≈ right / top  edge)

#define ADC_MID  2052.8 // ((ADC_MIN + ADC_MAX) / 2) | Actual value was measured with serial plotter
#define ADC_SPAN (ADC_MAX - ADC_MIN)         // ≈ 3675 // actual span of adc

uint16_t rawX, rawY;
float    u_x,    u_y;
bool    newData;

typedef struct // EMA filter Struct
{ float a, out; }
IFX_EMA;

IFX_EMA ema_x, ema_y; //EMA filters

int main() {
	init_clks();
	init_LPUART1();
	init_adc1();
	balance_timer();

	uint16_t x = 0;
	uint16_t y =0;

	float posX = 0;
	float posY = 0;

	ema_x.a   = 0.5f;
	ema_x.out = ADC_MID; //Initial X[0]
	ema_y.a   = 0.5f;
	ema_y.out = ADC_MID; //Initial Y[0]

	//linePattern      (50.0f,  -50.0f, 500, 3);

	while (1)	{
		//
		//		posX = 200 * ((float)x - ADC_MID) / ADC_SPAN;
		//		posY = 200 * ((float)y - ADC_MID) / ADC_SPAN;

		//		float x_set = 50 *cosf(w * t);
		//		float y_set = 50 * sinf(w * t);

		//
		/* Print to LPUART 1 921600 baud rate */

				if (newData) {

					float printUX = u_x*100 + ADC_MID;
					float printUY = u_y*100 + ADC_MID;
					newData = false;
					char str[80] = "$";
					char src[40];
					sprintf(src, "%d " , rawY);
					strcat(str, src);
					strcat(str, ";");
					txLPUART1(str);
					txLPUART1(src);             // one atomic write
				}
		//	    linePattern      (50.0f,  -50.0f, 200, 3);

		//				float u_x = pid_x(posX);
		//				float u_y = pid_y(posY);
		//
		//				uint32_t angleX = (uint32_t)(u_x + 90.0f);   // –45→45  →  45→135 deg
		//				uint32_t angleY = (uint32_t)(u_y + 90.0f);   // –45→45  →  45→135 deg
		//
		//				pwm_blue_x(angleX);
		//				pwm_green_y(angleY);
		//		//
		//		//		t += 0.05;
		//				delay_ms(50);

	}
	return 1;
}

void pwm_blue_x(uint32_t degrees){
	// Configure PB7 to be driven by the clock
	bitset(RCC->AHB2ENR, 1); 		// enable clock GPIOB
	bitclear(GPIOB->MODER, 14); 	// set PB7 to Alternate Function mode
	bitset(GPIOB->MODER, 15);
	GPIOB->AFR[0] &= ~(0xf << 28); 	// clear AFR
	bitset(GPIOB->AFR[0], 29); 		// set PB7 to Alternate Function 2 to connect to TIM4_CH2

	// Configure TIM4
	bitset(RCC->APB1ENR1, 2); 		// enable the clock for timer 4
	TIM4->PSC |= 160 - 1; 			// divide clock speed by 160
	TIM4->ARR = 2000 - 1; 			// set the auto load register
	bitclear(TIM4->CCMR1, 12); 		// set channel 2 to PWM mode 1, CCMR is set to output by default
	bitset(TIM4->CCMR1, 13);
	bitset(TIM4->CCMR1, 14);
	bitclear(TIM4->CCMR1, 24);
	TIM4->CCR2 = (degrees * 1) + 40;// set duty cycle (16 bit #, max val is 65535)
	bitset(TIM4->CCMR1, 11); 		// output compare 2 preload enable
	bitset(TIM4->CCER, 4); 			// enable capture/compare 2 output
	TIM4->CNT = 0; 					// reset counter current value
	TIM4->CR1|= 1; 					// enable the timer

}

void pwm_green_y(uint32_t degrees){
	// Configure PC7 to be driven by the clock
	bitset(RCC->AHB2ENR, 2); 		// enable clock GPIOC
	bitclear(GPIOC->MODER, 14); 	// set PC7 to Alternate Function mode
	bitset(GPIOC->MODER, 15);
	GPIOC->AFR[0] &= ~(0xf << 28); 	// clear AFR
	bitset(GPIOC->AFR[0], 29); 		// set PC7 to Alternate Function 2 to connect to TIM3_CH2

	// Configure TIM3
	bitset(RCC->APB1ENR1, 1); 		// enable the clock for timer 3
	TIM3->PSC |= 160 - 1; 			// divide clock speed by 160
	TIM3->ARR = 2000 - 1; 			// set the auto load register
	bitclear(TIM3->CCMR1, 12); 		// set channel 2 to PWM mode 1, CCMR is set to output by default
	bitset(TIM3->CCMR1, 13);
	bitset(TIM3->CCMR1, 14);
	bitclear(TIM3->CCMR1, 24);
	TIM3->CCR2 = (degrees * 1) + 40;// set duty cycle (16 bit #, max val is 65535)
	bitset(TIM3->CCMR1, 11); 		// output compare 2 preload enable
	bitset(TIM3->CCER, 4); 			// enable capture/compare 2 output
	TIM3->CNT = 0; 					// reset counter current value
	TIM3->CR1|= 1; 					// enable the timer
}

static inline float IFX_EMA_Update(IFX_EMA *f, float in) {
	// aplha aleady set
	f->out = f->a*in + (1.0f - f->a)*f->out; //EMA equation
	return f->out;
}

void balance_timer(){
	bitset(RCC-> APB1ENR1, 0); //Enables timer 2
	TIM2->PSC |= 160 - 1; 			// divide clock speed by 160
	TIM2->ARR = 5000 - 1; 			// set the auto load register
	TIM2->CNT = 0; 					// reset counter current value
	TIM2->DIER |= 1;
	TIM2->CR1|= 1;

	NVIC_SetPriority(TIM2_IRQn, 1); // set prio
	NVIC_EnableIRQ(TIM2_IRQn); //enable interrupt
}

void TIM2_IRQHandler(){
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR = 0;  // clear interrupt flag

		// 1) read, filter, & scale
		get_xy(&rawX, &rawY);
		float filtX = IFX_EMA_Update(&ema_x, (float)rawX);
		float filtY = IFX_EMA_Update(&ema_y, (float)rawY);

		float posX = 200*((float)filtX - ADC_MID)/ADC_SPAN;
		float posY = 200*((float)filtY - ADC_MID)/ADC_SPAN;

		// 2) PID on current global setpoints
		u_x = pid_x(posX );
		u_y = pid_y(posY );

		// 3) drive servos
		pwm_blue_x((uint32_t)(u_x + 90.0) );
		pwm_green_y((uint32_t)(u_y + 90.0) );

		newData = true;
	}
}

float pid_x(float currentX){
	/* Current Error - Proportional term (desired - where we are) */

	static float totalError = 0;
	static float previousError = 0;

	float e = currentX; //(0,0 - Yaxis)

	/* Accumulated Error - Integral term */
	totalError += e;

	/* Difference of Error - Derivative term */
	float deltaError = e - previousError;

	/* Also prepare for next iteration – set previous to Current Error */
	previousError = e;

	/* PID control variables */
	float Kp=0.20;
	float Ki=0.11;
	float Kd=0.06;
	float T =0.05;
	float u = 0;
	u = Kp * e + Ki * (totalError * T) + Kd * (deltaError / T);

	if(u > 30.0) u = 30.0;
	if(u < -30.0 ) u = -30.0;

	return u;

}
float pid_y(float currentY){
	/* Current Error - Proportional term (desired - where we are) */
	static float totalError = 0;
	static float previousError = 0;

	float e = -currentY; //(0,0 - Yaxis)

	/* Accumulated Error - Integral term */
	totalError += e;

	/* Difference of Error - Derivative term */
	float deltaError = e - previousError;

	/* Also prepare for next iteration – set previous to Current Error */
	previousError = e;

	/* PID control variables */
	float Kp=0.20;
	float Ki=0.11;
	float Kd=0.06;
	float T =0.05;
	float u = 0;
	u = Kp * e + Ki * (totalError * T) + Kd * (deltaError / T);

	if(u > 30)  u = 30;
	if(u < -30 ) u = -30;

	return u;
}

void linePattern(float rx, float ry, int wait_ms, int cycles) {
	for (int c = 0; c < cycles; c++) {
		// positive end
		float u_x = pid_x( rx );
		float u_y = pid_y( -ry );
		pwm_blue_x((uint32_t)(u_x + 90));
		pwm_green_y((uint32_t)(u_y + 90));
		delay_ms(wait_ms);
		// negative end
		u_x = pid_x( -rx );
		u_y = pid_y( -ry );
		pwm_blue_x((uint32_t)(u_x + 90));
		pwm_green_y((uint32_t)(u_y + 90));
		delay_ms(wait_ms);
	}
}


/**
 * Gets the x and y coordinates of the current touch panel
 * 		position. To measure x, makes y+ and y- tristate,
 * 		and creates a voltage divider between x+ and x-.
 * 		Then measures the voltage on Y+ ADC_CH1. To measure
 * 		y, makes x+ and x- tristate, creates a voltage divider
 * 		between y+ and y-. Then measures the X+ ADC_CH2.
 *
 * 		(purple) x+ ----> PD3 & ADC_CH2 PC1
 * 		(white)  y+ ----> PD4 & ADC_CH1 PC0
 * 		(yellow) x- ----> PD5
 * 		(black)  y- ----> PD6
 */
void get_xy(uint16_t* x, uint16_t* y) {
	RCC->AHB2ENR |= 1 << 3;		// turn on clock gpiod

	/* measure x axis voltage */
	bitclear(GPIOD->MODER, 8);		// set y+ PD4 tristate
	bitclear(GPIOD->MODER, 9);
	bitclear(GPIOD->ODR, 4);

	bitclear(GPIOD->MODER, 12);		// set y- PD6 tristate
	bitclear(GPIOD->MODER, 13);
	bitclear(GPIOD->ODR, 6);

	bitset(GPIOD->MODER, 6);		// set x+ PD3 out
	bitclear(GPIOD->MODER, 7);
	bitset(GPIOD->ODR, 3);			// set x+ PD3  high (3.3v)

	bitset(GPIOD->MODER, 10);		// set x- PD5 out
	bitclear(GPIOD->MODER, 11);
	bitclear(GPIOD->ODR, 5);		// set x- PD5  low

	bitset(ADC1->CR, 2);			// Start ADC conversion
	while (bitcheck(ADC1->ISR, 2) == 0) ; 	// wait for conversion complete
	*x = ADC1->DR & 0xfff;			// read adc val (clears EOC flag)

	/* measure y axis voltage */
	bitclear(GPIOD->MODER, 6);		// set x+ PD3 tristate
	bitclear(GPIOD->MODER, 7);
	bitclear(GPIOD->ODR, 3);

	bitclear(GPIOD->MODER, 10);		// set x- PD5 tristate
	bitclear(GPIOD->MODER, 11);
	bitclear(GPIOD->ODR, 5);

	bitset(GPIOD->MODER, 8);		// set y+ PD4 out
	bitclear(GPIOD->MODER, 9);
	bitset(GPIOD->ODR, 4);			// set D4 (y+) high

	bitset(GPIOD->MODER, 12);		// set y- PD6 out
	bitclear(GPIOD->MODER, 13);
	bitclear(GPIOD->ODR,6);			// set D6 (y-) low

	while (bitcheck(ADC1->ISR, 2) == 0) ; 	// wait for conversion complete
	*y = ADC1->DR & 0xfff;			// read adc val (clears EOC flag)
	delay_ms(1);
	bitclear(ADC1->ISR, 3);			// clear EOS flag
}

/* Recieves data from the LPUART Read Data Register if it is not empty */
uint16_t rxLPUART1() {
	uint16_t in_buf = 0;
	if (bitcheck(LPUART1->ISR, 5)) { 	// check if read data register not empty
		in_buf = LPUART1->RDR; 			// read from read data register
	}
	return in_buf;
}

/* Transmits data through LPUART Transmit data register while there is data */
void txLPUART1(char out_buf[]) {
	uint8_t i=0;
	while(out_buf[i] != '\0') {
		LPUART1->TDR = out_buf[i++];
		for(int j =0; j<10000; j++);
	}
}

// enable clocks
void init_clks() {
	RCC->APB1ENR1 |= 1 << 28; 	// enable power interface clock by setting PWREN bits
	RCC->APB1ENR2 |= 0x1;		// enable LPUART1EN clock
	RCC->CCIPR1 |= 0x800;		// 01 for HSI16 clock to be used for LPUART1
	RCC->CCIPR1 &= ~(0x400);
	RCC->CFGR |= 0x1;			// use HSI16 as SYSCLK
	RCC->CR |= 0x161;			// MSI clock enable; MSI = 4 MHz; HSI16 clock enable
	RCC->AHB2ENR |= 1<<6;		// enable clock to GPIOG
	bitset(RCC->APB1ENR2, 0);	// enable clock to LPUART1
}

/* PG7 is connected to LPUART tx PG8 is connect to LPUART rx */
void init_LPUART1() {
	PWR->CR2 |= 0x200;			// power up port g / enable VDDIO2 Independent I/O supply
	/* tx config */
	GPIOG->MODER &= ~(0x3<<14); // clear the two bits
	GPIOG->MODER |= 0x2 << 14;	// set mode to alternate function
	GPIOG->AFR[0] &= ~(0xf<<28);// clear 4 bits for PG7
	GPIOG->AFR[0] |= 0x8<<28;	// set PG7 to alternate function 8
	/* rx config */
	GPIOG->MODER &= ~(0x3<<16); // clear the two bits
	GPIOG->MODER |= 0x2 << 16;	// set mode to alternate function
	GPIOG->AFR[1] &= ~(0xf);	// clear 4 bits for PG8
	GPIOG->AFR[1] |= 0x8;		// set PG8 to alternate function 8

	// BRR = 256*16000000/57600 = 71111
	LPUART1->BRR = 4444; 				// set baud rate to 921600
	LPUART1->CR1 = 0xD; 				// 0x1101 --> TX, RX are enabled and UART is Enabled.
}

/**
 * Initialize ADC1 to read 2 values connected to CH1 and CH2.
 *  	CH1 -> PC0
 *  	CH2 -> PC1
 */
void init_adc1() {
	bitset(RCC->AHB2ENR, 13); 	// enable ADC clock
	RCC->CCIPR1 |= 0x3 << 28; 	// route SYSCLK HCLK to ADC

	bitclear(ADC1->CR, 29);		// exit deep power mode by setting DEEPPWD = 0 in control register
	bitset(ADC1->CR, 28); 		// turn on the ADC voltage reguator

	bitset(ADC1->CFGR, 12); 	// OVRMOD: Disable overrun mode (ADC keeps going even if user does not read)
	bitset(ADC1->ISR, 0); 		// ADC Ready

	delay_ms(10);				// wait for voltage regulator to stabilize

	/* Sequencer Setup */
	bitset(ADC1->SQR1, 0);		// set sequence length to 2
	bitset(ADC1->SQR1, 6);  	// set 1st conversion to ch1 0b0001
	bitset(ADC1->SQR1, 13);		// set 2nd conversion to ch2 0b0010

	ADC1->SMPR1 |= 0b000111111000;	// set sample speed to 700 ADC clock cycles

	ADC1->CR |= 1 ;				// enable ADC

	while (bitcheck(ADC1->ISR, 0) == 0);	// wait until ADC is ready
}
