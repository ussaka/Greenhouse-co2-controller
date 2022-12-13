/*
 ===============================================================================
 Name        : controller.cpp
 Author      : ..
 Version     : ..
 Copyright   : $(copyright)
 Description : ..
 ===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include "FreeRTOS.h"
#include "task.h"
#include "heap_lock_monitor.h"
#include "external/modbus/ModbusRegister.h"
#include "retarget_uart.h"

#include "Config.h"
#include "Disablers.h"


/* Standard includes. */
#include <string.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo Specific configs. */
#include "external/demo_config.h"

/* MQTT library includes. */
#include "core_mqtt.h"

/* Exponential backoff retry include. */
#include "external/backoff_algorithm.h"

/* Transport interface include. */
#include "using_plaintext.h"

// TODO: insert other definitions and declarations here
/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void) {
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}

// SW1 listener thread
static void vSendMQTT(void *pvParameters) {
	while (1) {
		vTaskDelay(50);
	}
}

static void idle_delay()
{
	vTaskDelay(1);
}

static void vMeasure(void *pvParameters) {
	(void) pvParameters;

	retarget_init();

	ModbusMaster node3(241); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister RH(&node3, 256, true);
	ModbusRegister temp(&node3, 257, true);
	float hum;
	float t = 0;

	while(true) {

		char buffer[32];

		vTaskDelay(2000);
		DisableScheduler d;

			hum = RH.read()/10.0;
			t = temp.read()/10.0;
			//snprintf(buffer, 32, "RH=%5.1f%%", rh);
			//printf("%s\n",buffer);
	}
}

static void vLcdUI(void *pvParameters) {
	while(1) {
		vTaskDelay(50);
	}
}


extern "C" {
  void vStartSimpleMQTTDemo( void ); // ugly - should be in a header
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats(void) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statictics collection */

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void) {
	prvSetupHardware();

	heap_monitor_setup();

	xTaskCreate(vSendMQTT, "vSendMQTT",
	configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(vMeasure, "vMeasure",
	configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 3UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(vLcdUI, "vLcdUI",
	configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t*) NULL);


	vStartSimpleMQTTDemo();

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
