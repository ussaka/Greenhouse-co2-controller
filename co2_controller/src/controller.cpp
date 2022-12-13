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
#include "ModbusMaster.h"
#include "ModbusRegister.h"

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
	}
}

static void vMeasure(void *pvParameters) {
	//ModbusMaster fan(1);
	//fan.begin(9600);

	ModbusMaster m();

	//ModbusMaster co2(240);
	//co2.begin(9600);

	//ModbusMaster hmp(241);
	//hmp.begin(9600);

//	ModbusRegister co2Data(&co2, 0x100, true);

	//	Relative humidity
	//ModbusRegister humidityData(&hmp, 0x100, true);
	//ModbusRegister temperatureData(&hmp, 0x101, true);


	int co2Value = 0;
	int rhValue = 0;
	int tempValue = 0;

	//Sleep(5);

	//if(hmpStatus.read())
	{
	//	Sleep(5);
		//tempValue = temperatureData.read() / 10;

	//	Sleep(5);
		//rhValue = humidityData.read() / 10;
	}

	//Sleep(5);
	//if(co2Status.read() == 0)
	{
		//Sleep(5);
		//co2Value = co2Data.read();
	}
	while(1) {

	}
}

static void vLcdUI(void *pvParameters) {
	while(1) {

	}
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
	configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(vLcdUI, "vLcdUI",
	configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t*) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
