/**
 * Self Balancing Platform
 * Authors: Michael Sportelli, Keegan Nelson
 *
 * Connections: X+ = PC0 ADC1, X- = PC1 ADC1, Y+ = PC2 ADC1, Y- = PC3 ADC1
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

void init_clks();
void init_LPUART1();
void init_adc1();
void init_dac1();
void init_comp1();
void init_redLED();
void pwm(uint32_t);
uint16_t rxLPUART1();
void txLPUART1();
float getVoltage(uint16_t);
uint16_t getADCVal();
void init_tim1();

int main() {
	init_clks();
	init_LPUART1();
	init_adc1();
	init_dac1();
	init_comp1();
	init_redLED();
	init_tim1(1000);						// initializes timer1 with a period of 250 ms

	/**
	 * This main function samples the first ADC value
	 * 		then will use a slightly higher value as the
	 * 		comparator trigger level, if it goes above
	 * 		the red led will light
	 */
	while (1)	{
//		if (bitcheck(COMP1->CSR, 30) == 1) {
//			bitset(GPIOA->ODR, 9); // red
//			delay_ms(10);
//		} else {
//			bitclear(GPIOA->ODR, 9); // red
//			delay_ms(10);
//		}
	}
	return 1;
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

void init_adc1() {
	bitset(RCC->AHB2ENR, 13); 	// enable ADC clock
	RCC->CCIPR1 |= 0x3 << 28; 	// route SYSCLK HCLK to ADC

	bitclear(ADC1->CR, 29);		// exit deep power mode by setting DEEPPWD = 0 in control register
	bitset(ADC1->CR, 28); 		// turn on the ADC voltage reguator

	bitset(ADC1->CFGR, 10);		// external trigger enable and polarity for rising edge
	bitclear(ADC1->CFGR, 11);

	bitclear(ADC1->CFGR, 6);	// external trigger EXT0 connected to TIM1_CH1
	bitclear(ADC1->CFGR, 7);
	bitclear(ADC1->CFGR, 8);
	bitclear(ADC1->CFGR, 9);

	bitset(ADC1->CFGR, 13); // enable continous mode

	bitset(ADC1->CFGR, 12); 	// OVRMOD: Disable overrun mode (ADC keeps going even if user does not read)
	bitset(ADC1->ISR, 0); 		// ADC Ready

	delay_ms(10);				// wait for voltage regulator to stabilize

	/* Sequencer Setup */
	bitset(ADC1->SQR1, 1); 		// set sequence length to 4
	bitset(ADC1->SQR1, 0);
	bitset(ADC1->SQR1, 6);  	// set 1st conversion to ch1 0b0001
	bitset(ADC1->SQR1, 13);		// set 2nd conversion to ch2 0b0010
	bitset(ADC1->SQR1, 18);		// set 3rd conversion to ch3 0b0011
	bitset(ADC1->SQR1, 19);
	bitset(ADC1->SQR1, 26);		// set 4th conversion to ch4 0b0100

	/* enable interrupt so that timer will trigger the ADC */
	NVIC_SetPriority(ADC1_2_IRQn, 0);
	NVIC_EnableIRQ(ADC1_2_IRQn);
	ADC1->IER |= 1 << 2; 		// enable EOC Interrupt
	ADC1->CR |= 1 ;				// enable ADC

	while (bitcheck(ADC1->ISR, 0) == 0);	// wait until ADC is ready
	bitset(ADC1->CR, 2);		// start conversion (wait for trigger)
}

/**
 * Function to be called when TIM1 goes high
 * 		formats the output voltage to be sent
 * 		to serial plotter through LPUART1 w/
 * 		921600 baud rate
 */
void ADC1_2_IRQHandler() {
	NVIC_DisableIRQ(ADC1_2_IRQn);
	//NVIC_DisableIRQ(TIM2_IRQn);
	uint16_t adc_val1 = ((ADC1->DR) & 0xfff);

	uint16_t adc_val2 = ((ADC1->DR) & 0xfff);

	uint16_t adc_val3 = ((ADC1->DR) & 0xfff);
	bitclear(ADC1->ISR, 3);		// clear EOS flag
//	uint16_t adc_val4 = ((ADC1->DR) & 0xfff);
	delay_ms(1000);

	char str[80] = "$";
	char src[40];
	sprintf(src, "%d %d %d", adc_val1, adc_val2, adc_val3);
	strcat(str, src);
	strcat(str, ";");
	txLPUART1(str);
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
 * Initialize DAC1 Clock, Configure PA4 (hooked up to DAC1_OUT1)
 */
void init_dac1() {
	bitset(RCC->APB1ENR1, 29); 	// initizize dac1 interface clock
	bitset(RCC->AHB2ENR, 0); 	// init GPIOA clock
	bitset(GPIOA->MODER, 6);	// set PA4 to analog mode
	bitset(GPIOA->MODER, 7);
	bitset(DAC1->MCR, 0);		// route dac1 to chip peripherals and external pin
	bitset(DAC1->CR, 0); 		// enable DAC1
}

/**
 * Initializes comparator
 */
void init_comp1() {
	bitset(RCC->APB2ENR, 0); 	// Turn on clock to comp1
	bitset(COMP1->CSR, 7);		// comp 1 inp = PB2 (1)
	bitclear(COMP1->CSR, 4);	// comp 1 inm = dac 1 (100)
	bitclear(COMP1->CSR, 5);
	bitset(COMP1->CSR, 6);
	bitset(COMP1->CSR, 0);		// enable comp1
}

/**
 * Initializes the red LED
 */
void init_redLED() {
	// red LED GPIOA Port 9
	bitset(RCC->AHB2ENR, 0); // enable clock
	bitset(GPIOA->MODER, 18); // set to output
	bitclear(GPIOA->MODER, 19);
}

// Use Timer 4 routed to LED_BLUE (PB7) (val should be 0 to 100)
void pwm(uint32_t val){
    // Use Timer 4:
    // This function demonstrates how to use the compare functionality of the timers.
    // The function will allow controlling the LED intensity using PWM.
    //
    // Useful link: https://www.youtube.com/watch?v=BtAi6-7Lnlw
    //
    // Steps to set up the timer in an output compare mode:
    //  1. Enable clock
    //  2. Set prescaler
    //  3. Set auto reload register (using the passed val)
    //  4. Set the Capture/Compare Mode Register to set output to PWM
    //  5. Set the match value to val (or something based on val?)
    //  6. Enable CHx compare mode which is connected to the PB7
    //  7. Reset the counter current value
    //  8. Enable the timer
    // No need to do anything else! The PWM of the PB7 is done automatically by the TIM4, allowing the CPU to perform other tasks.

    // Configure PB7 to be driven by the clock
	bitset(RCC->AHB2ENR, 1); // enable clock GPIOB
	bitclear(GPIOB->MODER, 14); // set PB7 to Alternate Function mode
	bitset(GPIOB->MODER, 15);
	GPIOB->AFR[0] &= ~(0xf << 28); // clear AFR
	bitset(GPIOB->AFR[0], 29); // set PB7 to Alternate Function 2 to connect to TIM4_CH2

    // Configure TIM4
	bitset(RCC->APB1ENR1, 2); // enable the clock for timer 4
	TIM4->PSC |= 4000 - 1; // divide clock speed by 4000
	TIM4->ARR = 100 - 1; // set the auto load register
	bitclear(TIM4->CCMR1, 12); // set channel 2 to PWM mode 1, CCMR is set to output by default
	bitset(TIM4->CCMR1, 13);
	bitset(TIM4->CCMR1, 14);
	bitclear(TIM4->CCMR1, 24);
	TIM4->CCR2 = val * 1; // set duty cycle (16 bit #, max val is 65535)
	bitset(TIM4->CCMR1, 11); // output compare 2 preload enable
	bitset(TIM4->CCER, 4); // enable capture/compare 2 output
	TIM4->CNT = 0; // reset counter current value
	TIM4->CR1|= 1; // enable the timer
}

/**
 * converts adc value to voltage
 */
float getVoltage(uint16_t adc_val) {
	// VREF=3.3V, step size is 3.3V/(2^12)=3.3/4096
	float voltage = adc_val * (3.3/4096);
	return voltage;
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

/* Initializes Timer 1 with given period and sets up the interrupt in NVIC to control ADC sample speed */
void init_tim1(int period) {
	bitset(RCC->APB2ENR, 11);	// enable Timer 1 clock
	TIM1->PSC = 16000 - 1; 	// Divided 16MHz source clk by 16000, for 1ms
	TIM1->ARR = period - 1; // Count 1ms PERIOD times
	TIM1->CCMR1 = 0x30;		// set output to toggle on match
	TIM1->CCR1 = 1;			// output will toggle when CNT==CCR
	bitset(TIM1->BDTR, 15);	// main output enable
	TIM1->CCER |= 1;		// enable CH1 compare mode
	TIM1->CNT = 0; 			// Clear counter
	TIM1->CR1 = 1; 			// Enable TIM1
}




