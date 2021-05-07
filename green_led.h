#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "system_MKL25Z4.h"

/* 
 * Green LED Pins
 */
#define PTE5_GLED1  	5
#define PTE4_GLED2  	4
#define PTE3_GLED3  	3
#define PTE2_GLED4  	2
#define PTB11_GLED5 	11
#define PTB10_GLED6 	10
#define PTB9_GLED7  	9
#define PTB8_GLED8  	8

#define MASK(x) (1 << (x))

unsigned int ledCounter = 0;

typedef enum led_order 
{ 
	 led_1 = 0,
	 led_2 = 1,
	 led_3 = 2,
	 led_4 = 3,
	 led_5 = 4,
	 led_6 = 5,
	 led_7 = 6,
	 led_8 = 7
 } led_order_t;

char led_mapping[8][2] = 
{
	{0, led_1},
	{1, led_2}, 
	{2, led_3},
	{3, led_4}, 
	{4, led_5},
	{5, led_6}, 
	{6, led_7},
	{7, led_8}
};

/*
 * Function:  initGLEDGPIO
 * --------------------
 * Initialises GPIO Pins for Green LEDs. 
 *
 */
void initGLEDGPIO(void) {
	// Enable clock to PORTB and PORTE
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTE_MASK));
	
	// Make pins GPIO
	PORTE->PCR[PTE5_GLED1] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE5_GLED1] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE4_GLED2] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE4_GLED2] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE3_GLED3] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE3_GLED3] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[PTE2_GLED4] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE2_GLED4] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB11_GLED5] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB11_GLED5] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB10_GLED6] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB10_GLED6] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB9_GLED7] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB9_GLED7] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[PTB8_GLED8] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB8_GLED8] |= PORT_PCR_MUX(1);
	
	// Set pins as output	
	PTB->PDDR |= (MASK(PTB11_GLED5) | 
								MASK(PTB10_GLED6) | 
								MASK(PTB9_GLED7) | 
								MASK(PTB8_GLED8));	
								
	PTE->PDDR |= (MASK(PTE5_GLED1) | 
								MASK(PTE4_GLED2) | 
								MASK(PTE3_GLED3) | 
								MASK(PTE2_GLED4));
}

/*
 * Function:  offGreenLEDs
 * --------------------
 * Turns off all Green LEDs.
 *
 */
void offGreenLEDs(void) {
	PTB->PCOR |= (MASK(PTB11_GLED5) | 
								MASK(PTB10_GLED6) | 
								MASK(PTB9_GLED7) | 
								MASK(PTB8_GLED8));	
	
	PTE->PCOR |= (MASK(PTE5_GLED1) | 
								MASK(PTE4_GLED2) | 
								MASK(PTE3_GLED3) | 
								MASK(PTE2_GLED4));
}

/*
 * Function:  onGreenLEDs
 * --------------------
 * Turns on all Green LEDs.
 *
 */
void onGreenLEDs(void) {
	PTB->PSOR |= (MASK(PTB11_GLED5) | 
								MASK(PTB10_GLED6) | 
								MASK(PTB9_GLED7) | 
								MASK(PTB8_GLED8));	
	
	PTE->PSOR |= (MASK(PTE5_GLED1) | 
								MASK(PTE4_GLED2) | 
								MASK(PTE3_GLED3) | 
								MASK(PTE2_GLED4));
}

/*
 * Function:  GLEDControl
 * --------------------
 * Control order in which Green LEDs are turned on.
 *
 * number:	The number of the led that is to be turned on
 *
 */
void GLEDControl(led_order_t number) {
	offGreenLEDs();
	
	switch(number) {
	case led_1:
		PTE->PSOR = MASK(PTE5_GLED1);
		break;
	
	case led_2:
		PTE->PSOR = MASK(PTE4_GLED2);
		break;

	case led_3:
		PTE->PSOR = MASK(PTE3_GLED3);
		break;

	case led_4:
		PTE->PSOR = MASK(PTE2_GLED4);
		break;

	case led_5:
		PTB->PSOR = MASK(PTB11_GLED5);
		break;

	case led_6:
		PTB->PSOR = MASK(PTB10_GLED6);
		break;		

	case led_7:
		PTB->PSOR = MASK(PTB9_GLED7);
		break;
	
	case led_8:
		PTB->PSOR = MASK(PTB8_GLED8);
		break;

	default:
		offGreenLEDs();
	}
}

/*
 * Function:  runGreenLEDs
 * --------------------
 * Turn on Green LEDs one-by-one in running sequence
 *
 */
void runGreenLEDs(void) {
	while(ledCounter <= 0x07) {
		ledCounter++;
		GLEDControl(led_mapping[ledCounter][1]);
		osDelay(100);
	} 
	ledCounter = 0;
}

/*
 * Function:  flashGreenLEDs
 * --------------------
 * Flash all Green LEDs.
 *
 */
void flashGreenLEDs(void) {
	for (int i = 0; i < 2; i++) {
		onGreenLEDs();
		osDelay(1000);
		offGreenLEDs();
		osDelay(1000);
	}
}
