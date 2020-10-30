#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"
#include "system_MKL25Z4.h"


/* UART */
#define BAUD_RATE 9600
#define PTE22_UART_TX 22
#define PTE23_UART_RX 23
#define UART2_INT_PRIO 128

#define MQ_SIZE 10
osMessageQueueId_t mqid_rx;


void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	// enable clock
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// enable appropriate multiplexing option
	//PORTE - alternative 4
	PORTE->PCR[PTE23_UART_RX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE23_UART_RX] |= PORT_PCR_MUX(4);
	
	// disable UART TX and RX first !!
	UART2->C2 &= ~UART_C2_RE_MASK;
	
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
	UART2->C2 |= UART_C2_RE_MASK;
	
	// set up interrupts for UART2_IRQn line
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
	// enable tx & rx interrupts in C2 register
	UART2->C2 |= UART_C2_RIE_MASK;
}

typedef struct {
	uint8_t data;
} myDataPkt;

void UART2_IRQHandler() {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	myDataPkt myData;
	
	/* FOR RECEIVING */
	// first, check if the rx data buffer is full
	if (UART2-> S1 & UART_S1_RDRF_MASK) {
		myData.data = UART2->D;
		osMessageQueuePut(mqid_rx, &myData, NULL, 0);
	}
	
	/* ERROR HANDLING */
	//if(UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// TODO: handle error, clear flag
			// delay(1000);
	//}
}
