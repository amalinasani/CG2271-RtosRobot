/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#include "green_led.h"
#include "red_led.h"
#include "motors.h"
#include "uart.h"



#define BIT4_MASK(x) (x & 0b00010000) //4th bit is the 'flag' for move cmd
#define MOVE_MASK(x) (x & 0b00011111) //check which movement direction
                                      //also checks for 4th bit, just as a safeguard
#define FORWARD      17 //0b 0001 0001
#define BACK         18 //0b 0001 0010
#define LEFT         20 //0b 0001 0100
#define RIGHT        24 //0b 0001 1000
#define BT_CONNECTED 1  //0b 0000 0001
#define RUN_COMPLETE 8  //0b 0000 1000

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

// GLOBAL
osThreadId_t tid_led_moving,
             tid_led_stopped;
osThreadId_t tid_move_forward,
             tid_move_back,
						 tid_turn_left,
						 tid_turn_right;
osSemaphoreId_t semid_led;

int INTERVAL = RLED_STOPPED_INTERVAL;


void tBrain(void *argument) {
	
		myDataPkt rx_data;
	
    for(;;) {
			
			osMessageQueueGet(mqid_rx, &rx_data, NULL, osWaitForever);

			if(BIT4_MASK(rx_data.data)) {
				
				INTERVAL = RLED_MOVING_INTERVAL;
				
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
			
			else {
				//if(rx_data == BT_CONNECTED) btconnect_actions();
				//else if (rx_data == RUN_COMPLETE) runcomplete_actions();
				INTERVAL = RLED_STOPPED_INTERVAL;
				stopMotors();
			}
				
		}
}

void tLedSemRelease(void *argument) {
	for(;;) {
		osSemaphoreRelease(semid_led);
		osDelay(500);
	}
}

void tLedControl (void *argument) {
	for (;;){
		osSemaphoreAcquire(semid_led, osWaitForever);
		if(INTERVAL == RLED_MOVING_INTERVAL) {
			runGreenLEDs();
			flashRedLEDs(RLED_MOVING_INTERVAL);
		}
		else if (INTERVAL == RLED_STOPPED_INTERVAL) {
			onGreenLEDs();
			flashRedLEDs(RLED_STOPPED_INTERVAL);
		}
	}
}


void tMotorForward (void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001,osFlagsWaitAny, osWaitForever);
		moveForward();
	}
}

void tMotorBack (void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001,osFlagsWaitAny, osWaitForever);
		moveBackward();
	}
}

 
void tMotorRight (void *argument) {
	for(;;) {
		osThreadFlagsWait(0x0001,osFlagsWaitAny, osWaitForever);
		turnLeft();
	}
}


void tMotorLeft (void *argument) {
	for (;;) {
		osThreadFlagsWait(0x0001,osFlagsWaitAny, osWaitForever);
		turnRight();
	}
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
	initGLEDGPIO();
	initRLEDGPIO();
	initUART2(BAUD_RATE);
	initMotorsPWM();
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	
	osThreadNew(tBrain, NULL, NULL);
	
	mqid_rx = osMessageQueueNew(MQ_SIZE, 1, NULL);
	
	semid_led = osSemaphoreNew(1, 0, NULL);
	osThreadNew(tLedSemRelease, NULL, NULL);
	tid_led_moving = osThreadNew(tLedControl, NULL, NULL);
	
	tid_move_forward = osThreadNew(tMotorForward, NULL, NULL);
	tid_move_back = osThreadNew(tMotorBack, NULL, NULL);
	tid_turn_left = osThreadNew(tMotorRight, NULL, NULL);
	tid_turn_right = osThreadNew(tMotorLeft, NULL, NULL);
	
	osKernelStart();                      // Start thread execution
  for (;;) {}
}


