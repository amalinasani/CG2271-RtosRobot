/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#include "globalutils.h"

 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void motor (void *argument) {
 
  for (;;) {
		if(!isQEmpty(&rxQ)) {
			rx_data = deqFromQ(&rxQ);
			if (BIT0_MASK(rx_data)) {
				isMoving = 1;
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
			}
			else {
				stopMotors();
			}
		}
	}
}

void led (void *argument) {
 
  for (;;) {
		if (isMoving) ledMoving();
		else ledNotMoving();
		osDelay(GLED_RATE);
	}
}
 
int main (void) {
 
  SystemCoreClockUpdate();
  initGLEDGPIO();
	initMotorsPWM();
	initUART2(BAUD_RATE);
 
  osKernelInitialize();
  osThreadNew(motor, NULL, NULL);
	osThreadNew(led, NULL, NULL);
  osKernelStart();
  for (;;) {}
}
