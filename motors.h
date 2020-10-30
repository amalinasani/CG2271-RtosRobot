#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "system_MKL25Z4.h"

/* MOTORS */
#define PTB1_RMotorF 1	// R, forwards
#define PTB0_RMotorB 0	// R, backwards
#define PTE30_LMotorF 30	// L, forwards
#define PTE29_LMotorB 29 // L, backwards

void initMotorsPWM(void) {
	// enable clock for PORTB and PORTA
	SIM_SCGC5 |= ((SIM_SCGC5_PORTB_MASK)|(SIM_SCGC5_PORTC_MASK)|(SIM_SCGC5_PORTE_MASK));
	
	// configure mode 3 for PWM pin operation
	// (Section 10.3.1, MUX)
	
	// 'alternative 3' is TPM1_CH1
	PORTB->PCR[PTB1_RMotorF] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_RMotorF] |= PORT_PCR_MUX(3);
	
	// 'alternative 3' is TPM1_CH0
	PORTB->PCR[PTB0_RMotorB] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_RMotorB] |= PORT_PCR_MUX(3);
	
	// 'alternative 3' is TPM0_CH3
	PORTE->PCR[PTE30_LMotorF] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE30_LMotorF] |= PORT_PCR_MUX(3);
	
	// 'alternative 3' is TPM0_CH2
	PORTE->PCR[PTE29_LMotorB] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_LMotorB] |= PORT_PCR_MUX(3);
	
	// enable clock for Timer 0, 1
	SIM->SCGC6 |= ((SIM_SCGC6_TPM0_MASK)|(SIM_SCGC6_TPM1_MASK));
	
	// select clock for TPM
	// (Section 12.2.3)
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	/* clk freq = 48MHz
	   prescaler = 128
		 MOD = 7500
		 48000000 / 128 / 7500 = 50Hz */

	// Timer 0
	TPM0->MOD = 7500; // period
	
	// EDGE-ALIGNED PWM -> CPWMS = 0
	// update TPM1_SC: CMOD = 01, PS = 111 (128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Timer 1
	TPM1->MOD = 7500; // period
	
	// EDGE-ALIGNED PWM -> CPWMS = 0
	// update TPM1_SC: CMOD = 01, PS = 111 (128)
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
}

void moveForward(void){
	
	// enable PWM on TPM1 Channel 1 -> PTB1
	// R Wheels, Forwards
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C1V = 0xEA6; // set duty cycle 50%
	
	// enable PWM on TPM0 Channel 3 -> PTE30
	// L Wheels, Forwards
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3V = 0xEA6; // set duty cycle 50%
}

void moveBackward(void){
	// enable PWM on TPM1 Channel 0 -> PTB0
	// R Wheels, Backwards
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C0V = 0xEA6; // set duty cycle 50%
	
	// enable PWM on TPM0 Channel 2 -> PTE29
	// L Wheels, Backwards
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C2V = 0xEA6; // set duty cycle 50%
}

void turnLeft(void){
	// enable PWM on TPM1 Channel 1 -> PTB1
	// R Wheels, Forwards
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C1V = 0xEA6; // set duty cycle 50%
	
	// enable PWM on TPM0 Channel 3 -> PTE30
	// L Wheels, Forwards
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
}

void turnRight(void){
	// enable PWM on TPM1 Channel 1 -> PTB1
	// R Wheels, Forwards
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	
	// enable PWM on TPM0 Channel 3 -> PTE30
	// L Wheels, Forwards
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3V = 0xEA6; // set duty cycle 50%
}

void stopMotors(void){
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));

	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));

	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
}
