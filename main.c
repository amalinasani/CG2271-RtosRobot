#include "green_led.h"
#include "red_led.h"
#include "motors.h"
#include "uart.h"
#include "buzzer.h"

// Mask to check for 4th bit, the 'flag' for move command
#define BIT4_MASK(x) (x & 0b00010000) 

// Mask to determine movement direction. Checks which of the 0th to 3rd bit is set
// Also checks for 4th bit as a safeguard
#define MOVE_MASK(x) (x & 0b00011111) 
																			
// Receiving data packets 
#define FORWARD    17 	//0b 0001 0001
#define BACK       18 	//0b 0001 0010
#define LEFT       20 	//0b 0001 0100
#define RIGHT      24 	//0b 0001 1000
#define BT_CONNECT 1  	//0b 0000 0001
#define RUN_END    8  	//0b 0000 1000

// Buzzer audio states
#define STATE_BT  1 //when Bluetooth connection has just been established
#define STATE_END 2 //when the challenge run ends
#define STATE_RUN 0 //all other time periods

// Global variables 
int RLED_FREQ = RLED_STOP_FREQ;
int AUDIO_STATE = STATE_RUN;

// Motor movement thread IDs
osThreadId_t tid_move_forward,
             tid_move_back,
						 tid_turn_left,
						 tid_turn_right;
						 
// LED thread and semaphore IDs	 
osThreadId_t tid_led_control;
osSemaphoreId_t semid_led;

/*
 * Thread:  tBrain
 * --------------------
 * Thread that parses RX data packets,
 * sets respective flags, and calls relevant functions.
 *
 */
void tBrain(void *argument) {
	myDataPkt rx_data;

	for(;;) {			
		osMessageQueueGet(mqid_rx, 
											&rx_data, 
											NULL, 
											osWaitForever);

		if(BIT4_MASK(rx_data.data)) {
			RLED_FREQ = RLED_MOVE_FREQ;
			
			switch(MOVE_MASK(rx_data.data)) {
			case(FORWARD):
				osThreadFlagsSet(tid_move_forward, 0x0001);
				break;
			case(BACK):
				osThreadFlagsSet(tid_move_back, 0x0001);
				break;
			case(LEFT):
				osThreadFlagsSet(tid_turn_left, 0x0001);
				break;
			case(RIGHT):
				osThreadFlagsSet(tid_turn_right, 0x0001);
				break;
			default:
				stopMotors();
			}
		}
		else if (rx_data.data == BT_CONNECT) {
			AUDIO_STATE = STATE_BT;
			flashGreenLEDs();
		}
		else if (rx_data.data == RUN_END){
			AUDIO_STATE = STATE_END;
		}
		else {
			RLED_FREQ = RLED_STOP_FREQ;
			stopMotors();
		}
	}
}

/*
 * Thread:  tAudio
 * --------------------
 * Thread that controls music functions. 
 *
 */
void tAudio(void *argument) {
	int thisNote = 0;
	int notes = sizeof(miiMelody) / sizeof(miiMelody[0]) / 2;
	
	for(;;) {
		switch (AUDIO_STATE) {
		case STATE_RUN: 
			playSong(thisNote);
			if (thisNote > notes * 2)
				thisNote = 0;
			else 
				thisNote += 2;
			break;
		
		case STATE_BT:
			playBtSong();
			AUDIO_STATE = STATE_RUN;
			break;
		
		case STATE_END:
			playEndSong();
			AUDIO_STATE = STATE_RUN;
			break;
		}
	}
}

/*
 * Thread:  tLedControl
 * --------------------
 * Thread that controls LED functions.
 *
 */
void tLedControl (void *argument) {
	for (;;) {
		osSemaphoreAcquire(semid_led, osWaitForever);
		
		if (RLED_FREQ == RLED_MOVE_FREQ) {
			runGreenLEDs();
			flashRedLEDs(RLED_MOVE_FREQ);
		} 
		else if (RLED_FREQ == RLED_STOP_FREQ) {
			onGreenLEDs();
			flashRedLEDs(RLED_STOP_FREQ);
		}
	}
}

/*
 * Thread:  tLedSemRelease
 * --------------------
 * Thread that releases semaphore for LED control at periodic intervals.
 *
 */
void tLedSemRelease (void *argument) {
	for (;;) {
		osSemaphoreRelease(semid_led);
		osDelay(500);
	}
}

/*
 * Thread:  tMotorForward
 * --------------------
 * Thread to control "forwards" movement.
 *
 */
void tMotorForward (void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001, 
											osFlagsWaitAny, 
											osWaitForever);
		moveForward();
	}
}

/*
 * Thread:  tMotorBack
 * --------------------
 * Thread to control "backwards" movement.
 *
 */
void tMotorBack (void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001,
											osFlagsWaitAny, 
											osWaitForever);
		moveBackward();
	}
}
 
/*
 * Thread:  tMotorRight
 * --------------------
 * Thread to control "turn left" movement.
 *
 */
void tMotorRight (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x0001,
											osFlagsWaitAny, 
											osWaitForever);
		turnLeft();
	}
}

/*
 * Thread:  tMotorLeft
 * --------------------
 * Thread to control "turn right" movement.
 *
 */
void tMotorLeft (void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001,
											osFlagsWaitAny, 
											osWaitForever);
		turnRight();
	}
}

/*
 * Function:	main
 * --------------------
 * Main function that initialises all components,
 * osKernel, and threads. 
 * Starts the initialised kernel.
 *
 */
int main (void) {
  SystemCoreClockUpdate();

	initGLEDGPIO();
	initRLEDGPIO();
	initUART2(BAUD_RATE);
	initMotors();
	initBuzzer();
	
  osKernelInitialize();
	mqid_rx = osMessageQueueNew(MQ_SIZE, 1, NULL);
	semid_led = osSemaphoreNew(1, 0 , NULL);

	osThreadNew(tBrain, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);	
	tid_led_control = osThreadNew(tLedControl, NULL, NULL);
	osThreadNew(tLedSemRelease, NULL, NULL);
	tid_move_forward = osThreadNew(tMotorForward, NULL, NULL);
	tid_move_back = osThreadNew(tMotorBack, NULL, NULL);
	tid_turn_left = osThreadNew(tMotorRight, NULL, NULL);
	tid_turn_right = osThreadNew(tMotorLeft, NULL, NULL);
	
	osKernelStart();
  for (;;) {}
}


