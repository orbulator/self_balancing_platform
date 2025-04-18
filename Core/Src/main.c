/**
 * Self Balancing Platform
 * Authors: Michael Sportelli, Keegan Nelson
 *
 * Connections:
 *
 */

#include "stm32l552xx.h"
#include "stdio.h"
#include "math.h"
#include "string.h"


// Macros
#define bitclear(word, idx)  (word &= ~(1<<idx)) // clears bit #
#define bitset(word, idx)    (word |= (1<<idx) ) // sets bit #
#define bitflip(word, idx)   (word ^= (1<<idx)) // flips bit #
#define bitcheck(word, idx)  ((word>>idx) & 1) //checks bit number 0 means clear
#define delay_ms(val) for (int i = 0; i < (1600 * val); i++) {} // delays x milliseconds

/* Helper functions */
void pwm_blue_x(uint32_t);
void pwm_green_y(uint32_t);
void get_xy(uint16_t *, uint16_t *);
uint16_t rxLPUART1();
void txLPUART1();

/* Init Functions */
void init_clks();
void init_LPUART1();
void init_adc1();

int main() {
	init_clks();
	init_LPUART1();
	init_adc1();

	uint16_t x = 0; // x coordinate of touch panel
	uint16_t y = 0; // y coordinate of touch panel

	while (1)	{

		get_xy(&x, &y);

		/* Print to LPUART 1 921600 baud rate */
//		char str[80] = "$";
//		char src[40];
//		sprintf(src, "%d %d", x, y);
//		strcat(str, src);
//		strcat(str, ";");
//		txLPUART1(str);


			pwm_blue_x(x / 21);
			pwm_green_y(y / 21);
			delay_ms(50);

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

void pwm_control_x(int16_t currentX){
	/* Current Error - Proportional term (desired - where we are) (0,0 - Xaxis)*/
	int16_t e = 0 - currentX; //(0,0 - Xaxis)

	/* Accumulated Error - Integral term */
	int16_t totalError = 0;
	totalError += 0;

	/* Also prepare for next iteration – set previous to Current Error */
	int16_t previousError = e;

	/* Difference of Error - Derivative term */
	int16_t deltaError = e - previousError;

	/* PID control */
	int8_t Kp=0;
	int8_t Ki=0;
	int8_t Kd=0;
	int8_t T =0;
	int16_t u = 0;
	u = Kp * e + Ki * (totalError * T) + Kd * (deltaError / T);

	return u;

}

void pwm_control_y(int16_t currentY){
	/* Current Error - Proportional term (desired - where we are) */
	int16_t e = 0 - currentY; //(0,0 - Yaxis)

	/* Accumulated Error - Integral term */
	int16_t totalError = 0;
	totalError += 0;

	/* Also prepare for next iteration – set previous to Current Error */
	int16_t previousError = e;

	/* Difference of Error - Derivative term */
	int16_t deltaError = e - previousError;

	/* PID control */
	int8_t Kp=0;
	int8_t Ki=0;
	int8_t Kd=0;
	int8_t T = 0;
	int16_t u = 0;
	u = Kp * e + Ki * (totalError * T) + Kd * (deltaError / T);

	return u;

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





