#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "system_MKL25Z4.h"

/* GREEN LEDs */
#define PTA1_GLED1 1
#define PTA2_GLED2 2
#define PTD4_GLED3 4
#define PTA12_GLED4 12
#define PTA4_GLED5 4
#define PTA5_GLED6 5
#define PTC8_GLED7 8
#define PTC9_GLED8 9

#define GLED_RATE 100

#define MASK(x) (1 << (x))

unsigned int ledCounter = 0;


typedef enum led_order { led_1 = 0,
												 led_2 = 1,
	                       led_3 = 2,
	                       led_4 = 3,
	                       led_5 = 4,
	                       led_6 = 5,
	                       led_7 = 6,
	                       led_8 = 7
                       } led_order_t;

char led_mapping[8][2] = {{0, led_1},
													{1, led_2}, 
													{2, led_3},
													{3, led_4}, 
													{4, led_5},
													{5, led_6}, 
													{6, led_7},
													{7, led_8}
												};

void initGLEDGPIO(void){
	// Enable clock to Port B and Port D
	SIM->SCGC5 |= ((SIM_SCGC5_PORTA_MASK) | (SIM_SCGC5_PORTC_MASK)| (SIM_SCGC5_PORTD_MASK));
	
	// Make pins GPIO
	PORTA->PCR[PTA1_GLED1] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA1_GLED1] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA2_GLED2] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA2_GLED2] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[PTD4_GLED3] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD4_GLED3] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA12_GLED4] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA12_GLED4] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA4_GLED5] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA4_GLED5] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[PTA5_GLED6] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA5_GLED6] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC8_GLED7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC8_GLED7] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[PTC9_GLED8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC9_GLED8] |= PORT_PCR_MUX(1);
	
	// Set pins as output
	PTA->PDDR |= (MASK(PTA1_GLED1) | MASK(PTA2_GLED2) | MASK(PTA4_GLED5) | MASK(PTA5_GLED6) | MASK(PTA12_GLED4));	
	PTD->PDDR |= MASK(PTD4_GLED3);
	PTC->PDDR |= (MASK(PTC8_GLED7) | MASK(PTC9_GLED8));
}

void offGreenLEDs(void){
	PTA->PCOR |= (MASK(PTA1_GLED1) | MASK(PTA2_GLED2) | MASK(PTA4_GLED5) | MASK(PTA5_GLED6) | MASK(PTA12_GLED4));	
	PTD->PCOR |= MASK(PTD4_GLED3);
	PTC->PCOR |= (MASK(PTC8_GLED7) | MASK(PTC9_GLED8));
}

void onGreenLEDs(void) {
	PTA->PSOR |= (MASK(PTA1_GLED1) | MASK(PTA2_GLED2) | MASK(PTA4_GLED5) | MASK(PTA5_GLED6) | MASK(PTA12_GLED4));	
	PTD->PSOR |= MASK(PTD4_GLED3);
	PTC->PSOR |= (MASK(PTC8_GLED7) | MASK(PTC9_GLED8));
}

void GLEDControl(led_order_t number){
	offGreenLEDs();
	switch(number){
		case led_1:
			PTA->PSOR = MASK(PTA1_GLED1);
			break;
		
		case led_2:
			PTA->PSOR = MASK(PTA2_GLED2);
			break;
	
		case led_3:
			PTD->PSOR = MASK(PTD4_GLED3);
			break;

		case led_4:
			PTA->PSOR = MASK(PTA12_GLED4);
			break;

		case led_5:
			PTA->PSOR = MASK(PTA4_GLED5);
			break;
	
		case led_6:
			PTA->PSOR = MASK(PTA5_GLED6);
			break;		

		case led_7:
			PTC->PSOR = MASK(PTC8_GLED7);
			break;
		
		case led_8:
			PTC->PSOR = MASK(PTC9_GLED8);
			break;
	
		default:
			offGreenLEDs();
	}
}

void runGreenLEDs(void) {
	ledCounter++;
	GLEDControl(led_mapping[ledCounter][1]);
	if (ledCounter > 0x07) ledCounter = 0;
}
