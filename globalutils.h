#include "MKL25Z4.h"
#include "system_MKL25Z4.h"

/* GLOBAL STUFF */
#define MASK(x) (1 << (x))
unsigned int ledCounter = 0;
volatile uint8_t rx_data;
volatile int isMoving = 0;

/* UART */
#define BAUD_RATE 9600
#define PTE22_UART_TX 22
#define PTE23_UART_RX 23
#define UART2_INT_PRIO 128

#define FORWARD 2  //0b0000 0010
#define BACK    4  //0b0000 0100
#define LEFT    8  //0b0000 1000
#define RIGHT   16 //0b0001 0000

#define MOVE_MASK(x) (x & 0X1E) //x & 0b0001 1110
#define BIT0_MASK(x) (x & 0x01) //check if ON bit is enabled

// for UART data buffers
#define Q_SIZE 5
#define INIT_VAL 0

typedef struct {
	unsigned char Data[Q_SIZE];
	unsigned int Head; // points to oldest data element
	unsigned int Tail; // points to next free space
	unsigned int Size; // number of elements in queue
} Q_T;

Q_T txQ, rxQ;

void initQ(Q_T *q) {
	unsigned int i;
	for (i = 0; i < Q_SIZE; i++) {
		q->Data[i] = INIT_VAL; // init all data elements to zero
	}
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int isQEmpty(Q_T *q) {
	return q->Size == 0;
}

int isQFull(Q_T *q) {
	return q->Size == Q_SIZE;
}

int enqToQ(Q_T *q, unsigned char d) {
	if(!isQFull(q)) {
		q->Data[q->Tail++] = d;
		q->Tail %= Q_SIZE;
		q->Size++;
		return 1;
	}
	else return 0;
}

unsigned char deqFromQ(Q_T *q) {
	unsigned char t = INIT_VAL;
	if(!isQEmpty(q)) {
		t = q->Data[q->Head];
		q->Data[q->Head++] = INIT_VAL;
		q->Head %= Q_SIZE;
		q->Size--;
	}
	return t;
}

void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	// enable clock
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// enable appropriate multiplexing option
	//PORTE - alternative 4
	PORTE->PCR[PTE22_UART_TX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE22_UART_TX] |= PORT_PCR_MUX(4);
	PORTE->PCR[PTE23_UART_RX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE23_UART_RX] |= PORT_PCR_MUX(4);
	
	// disable UART TX and RX first !!
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	// SBR[12:0] = (bus clock) / (baud rate * 16)
	// bus clock: CPUclk / 2, that UART peripheral subsystem runs on
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2;
	divisor = (bus_clock) / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// set these 3 registers to zero
	// may change depending on ur preferred settings
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	// re-enable UART TX and RX
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	// set up interrupts for UART2_IRQn line
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	// enable tx & rx interrupts in C2 register
	UART2->C2 |= UART_C2_TIE_MASK | UART_C2_RIE_MASK;
	initQ(&txQ);
	initQ(&rxQ);
}

void UART2_IRQHandler() {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	/* FOR TRANSMISSION */
	// first, check if the tx data buffer is empty
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// then, check if the queue has data waiting to be sent
		if(!isQEmpty(&txQ)) {
			UART2->D = deqFromQ(&txQ); // if so, dequeue data and send
		}
		else {
			UART2->C2 &= ~UART_C2_TIE_MASK; //else, disable tx
		}
	}
	
	/* FOR RECEIVING */
	// first, check if the rx data buffer is full
	if (UART2-> S1 & UART_S1_RDRF_MASK) {
		// then, check if the queue is not full i.e. can still receive data
		if (!isQFull(&rxQ)) {
			enqToQ(&rxQ, UART2->D); // if not full, enqueue received data
		}
//		else {
//			delay(1000); // TODO: handle error
//		}
	}
	
	/* ERROR HANDLING */
	if(UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// TODO: handle error, clear flag
			// delay(1000);
	}
}






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
	
	isMoving = 0;
	
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

void ledControl(led_order_t number){
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
	ledControl(led_mapping[ledCounter][1]);
	if (ledCounter > 0x07) ledCounter = 0;
}


void ledMoving(void) {
	runGreenLEDs();
	//TODO: implement moving red LEDs behaviour
}

void ledNotMoving(void) {
	onGreenLEDs();
	//TODO: implement NOT moving red LEDs behaviour
}


