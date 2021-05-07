#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "system_MKL25Z4.h"

/* 
 * Red LED Pins
 *
 */
#define PTA1_RLED1  	1
#define PTA2_RLED2  	2
#define PTD4_RLED3  	4
#define PTA12_RLED4 	12
#define PTA4_RLED5  	4
#define PTA5_RLED6  	5
#define PTC8_RLED7  	8
#define PTC9_RLED8  	9

/* 
 * Red LED Flash Frequencies
 *
 */
#define RLED_MOVE_FREQ 500 //when moving
#define RLED_STOP_FREQ 250 //when stationary

#define MASK(x) (1 << (x))

/*
 * Function:  initRLEDGPIO
 * --------------------
 * Initialises GPIO Pins for Red LEDs. 
 *
 */
void initRLEDGPIO(void) {
	// Enable clock to PORTA, PORTC and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK)| (SIM_SCGC5_PORTD_MASK));
	
	// Make pins GPIO
	PORTA->PCR[PTA1_RLED1] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA1_RLED1] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA2_RLED2] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA2_RLED2] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[PTD4_RLED3] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD4_RLED3] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA12_RLED4] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA12_RLED4] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA4_RLED5] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA4_RLED5] |= PORT_PCR_MUX(1);

	PORTA->PCR[PTA5_RLED6] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA5_RLED6] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC8_RLED7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC8_RLED7] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC9_RLED8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC9_RLED8] |= PORT_PCR_MUX(1);
	
	// Set pins as output
	PTA->PDDR |= (MASK(PTA1_RLED1) | 
								MASK(PTA2_RLED2) | 
								MASK(PTA4_RLED5) | 
								MASK(PTA5_RLED6) | 
								MASK(PTA12_RLED4));
								
	PTC->PDDR |= (MASK(PTC8_RLED7) | 
								MASK(PTC9_RLED8));
								
	PTD->PDDR |= 	MASK(PTD4_RLED3);
}

/*
 * Function:  offRedLEDs
 * --------------------
 * Turns off all Red LEDs.
 *
 */
void offRedLEDs(void) {
	PTA->PCOR |= (MASK(PTA1_RLED1) | 
								MASK(PTA2_RLED2) | 
								MASK(PTA4_RLED5) | 
								MASK(PTA5_RLED6) | 
								MASK(PTA12_RLED4));
	
	PTC->PCOR |= (MASK(PTC8_RLED7) | 
								MASK(PTC9_RLED8));
	
	PTD->PCOR |= 	MASK(PTD4_RLED3);
}

/*
 * Function:  onRedLEDs
 * --------------------
 * Turns on all Red LEDs.
 *
 */
void onRedLEDs(void) {
	PTA->PSOR |= (MASK(PTA1_RLED1) | 
								MASK(PTA2_RLED2) | 
								MASK(PTA4_RLED5) | 
								MASK(PTA5_RLED6) | 
								MASK(PTA12_RLED4));
	
	PTC->PSOR |= (MASK(PTC8_RLED7) | 
								MASK(PTC9_RLED8));
	
	PTD->PSOR |= 	MASK(PTD4_RLED3);
}

/*
 * Function:  flashRedLEDs
 * --------------------
 * Flash all Red LEDs.
 *
 * interval: Time in between flashes
 *
 */
void flashRedLEDs(int interval) {
		onRedLEDs();
		osDelay(interval);
		offRedLEDs();
		osDelay(interval);
}
