#include "MKL25Z4.h"                    // Device header
#include "system_MKL25Z4.h"             // Keil::Device:Startup

volatile uint8_t rx_data;

/* UART */
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

#define FORWARD 2  //0b0000 0010
#define BACK    4  //0b0000 0100
#define LEFT    8  //0b0000 1000
#define RIGHT   16 //0b0001 0000
#define MOVE_MASK(x) (x & 0X1E) //x & 0b0001 1110
#define BIT0_MASK(x) (x & 0x01) //check if ON bit is enabled

// #define RED_LED   18
// #define GREEN_LED 19
// #define BLUE_LED  1
#define MASK(x) (1 << (x))

// for uart data buffers
#define Q_SIZE 5
#define INIT_VAL 0

/* MOTORS */
#define PTB1_Pin 1	// R, forwards
#define PTB0_Pin 0	// R, backwards
#define PTE30_Pin 30	// L, forwards
#define PTE29_Pin 29 // L, backwards

static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

void delay_mult100(volatile uint32_t nof) {
	for (int i = 0; i < 100; i++) 
		delay(nof);
}

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
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
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
			delay(1000);
	}
}

void initPWM(void) {
	// enable clock for PORTB and PORTA
	SIM_SCGC5 |= ((SIM_SCGC5_PORTB_MASK)|(SIM_SCGC5_PORTC_MASK)|(SIM_SCGC5_PORTE_MASK));
	
	// configure mode 3 for PWM pin operation
	// (Section 10.3.1, MUX)
	
	// 'alternative 3' is TPM1_CH1
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	// 'alternative 3' is TPM1_CH0
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	// 'alternative 3' is TPM0_CH3
	PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(3);
	
	// 'alternative 3' is TPM0_CH2
	PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);
	
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


int main(void) {
	char i = 0;
	SystemCoreClockUpdate();
	initPWM();
	initUART2(BAUD_RATE);
	
	while(1) {
		
		if(!isQEmpty(&rxQ)) {
			rx_data = deqFromQ(&rxQ);
			if (BIT0_MASK(rx_data))
				switch(MOVE_MASK(rx_data)) {
					case (FORWARD):
						moveForward();
						break;
					case (BACK):
						moveBackward();
						break;
					case (LEFT):
						turnLeft();
						break;
					case (RIGHT):
						turnRight();
						break;
					default:
						stopMotors();
				}
			else {
				stopMotors();
			}
		}
		
	}
}