#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "system_MKL25Z4.h"

/* 
 * Motor Pins
 */
#define PTB1_RMotorF  1	 // R, Forward
#define PTB0_RMotorB  0	 // R, Backward
#define PTE30_LMotorF 30 // L, Forward
#define PTE29_LMotorB 29 // L, Backward

/* 
 * Duty cycle values
 */
uint32_t PWM_VAL_100 = 0x1D4C; 	// 100% Duty Cycle
uint32_t PWM_VAL_10 = 0x2EE; 		// 10% 	Duty Cycle

/*
 * Function:  initMotors
 * --------------------
 * Initialises GPIO Pins for Motors. 
 *
 */
void initMotors(void) {
	// Enable clock for PORTB, PORTC, PORTE
	SIM_SCGC5 |= ((SIM_SCGC5_PORTB_MASK)|
								(SIM_SCGC5_PORTC_MASK)|
								(SIM_SCGC5_PORTE_MASK));
	
	// 'Alternative 3' - TPM1_CH1
	PORTB->PCR[PTB1_RMotorF] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_RMotorF] |= PORT_PCR_MUX(3);
	
	// 'Alternative 3' - TPM1_CH0
	PORTB->PCR[PTB0_RMotorB] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_RMotorB] |= PORT_PCR_MUX(3);
	
	// 'Alternative 3' - TPM0_CH3
	PORTE->PCR[PTE30_LMotorF] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE30_LMotorF] |= PORT_PCR_MUX(3);
	
	// 'Alternative 3' - TPM0_CH2
	PORTE->PCR[PTE29_LMotorB] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_LMotorB] |= PORT_PCR_MUX(3);
	
	// Enable clock for TPM0 and TPM1
	SIM->SCGC6 |= ((SIM_SCGC6_TPM0_MASK)|(SIM_SCGC6_TPM1_MASK));
	
	// Select clock for TPM
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	// TPM0 
	TPM0->MOD = 7500; 
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// TPM1
	TPM1->MOD = 7500;
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
}

/*
 * Function:  moveForward
 * --------------------
 * Enables PWM to left and right motors to move forward.
 *
 */
void moveForward(void) {
	// Enable PWM on TPM1 Channel 1 -> PTB1
	// R Wheels, Forwards
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C1V = PWM_VAL_100; 
	
	// Enable PWM on TPM0 Channel 3 -> PTE30
	// L Wheels, Forwards
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3V = PWM_VAL_100; 
}

/*
 * Function:  moveBackward
 * --------------------
 * Enables PWM to left and right motors to move backward.
 *
 */
void moveBackward(void) {
	// Enable PWM on TPM1 Channel 0 -> PTB0
	// R Wheels, Backwards
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C0V = PWM_VAL_100; 
	
	// Enable PWM on TPM0 Channel 2 -> PTE29
	// L Wheels, Backwards
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C2V = PWM_VAL_100; 
}

/*
 * Function:  turnLeft
 * --------------------
 * Enables 10% PWM duty cycle to left motors and 
 * 100% PWM duty cycle to right motors to turn the robot left.
 *
 */
void turnLeft(void) {
	// Enable PWM on TPM1 Channel 1 -> PTB1
	// R Wheels, Forwards
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C1V = PWM_VAL_100; 
	
	// Enable PWM on TPM0 Channel 3 -> PTE30
	// L Wheels, Forwards
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3V = PWM_VAL_10; 
}

/*
 * Function:  turnRight
 * --------------------
 * Enables 10% PWM duty cycle to right motors and 
 * 100% PWM duty cycle to left motors to turn the robot right.
 *
 */
void turnRight(void) {
	// Enable PWM on TPM1 Channel 1 -> PTB1
	// R Wheels, Forwards
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM1_C1V = PWM_VAL_10; 
	
	// Enable PWM on TPM0 Channel 3 -> PTE30
	// L Wheels, Forwards
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | 
	               (TPM_CnSC_ELSA_MASK) |
								 (TPM_CnSC_MSB_MASK)  |
								 (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	TPM0_C3V = PWM_VAL_100; 
}

/*
 * Function:  stopMotors
 * --------------------
 * Disables PWM to left and right motors to stop all movement.
 *
 */
void stopMotors(void) {
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
