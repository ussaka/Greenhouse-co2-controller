/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
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
#include "semphr.h"
#include "heap_lock_monitor.h"
#include "retarget_uart.h"

#include "ModbusRegister.h"
#include "DigitalIoPin.h"
#include "LiquidCrystal.h"

#include "Config.h"
#include "menu/Menu.h"
#include "menu/NumericProperty.h"
#include "Disablers.h"

// TODO: insert other definitions and declarations here

/* The following is required if runtime statistics are to be collected
 * Copy the code to the source file where other you initialize hardware */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statictics collection */

static void setupGPIOInterrupts(void) {
	/* Initialise PININT driver */
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	#define SIGA 0, 5
	#define SIGB 0, 6

	#define BUTTON_SELECT

	/* Set pins back to GPIO and configure as inputs*/
	// SIGA
	Chip_IOCON_PinMuxSet(LPC_IOCON, SIGA,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP));
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, SIGA);

	// SIGB
	Chip_IOCON_PinMuxSet(LPC_IOCON, SIGB,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP));
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, SIGB);

	/* Enable PININT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PININT);

	/* Reset the PININT block */
	Chip_SYSCTL_PeriphReset(RESET_PININT);

	/* Configure interrupt channels for the GPIO pins in INMUX block */
	Chip_INMUX_PinIntSel(0, SIGA); // SIGA

	/* Configure channel interrupts as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(0));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(0));

	/* Enable interrupts in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_SetPriority(PIN_INT0_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
}

static void idle_delay()
{
	vTaskDelay(1);
}

// SW1 listener thread
static void vSendMQTT(void *pvParameters) {
	while (1) {
		vTaskDelay(50);
	}
}


static void vMeasure(void *pvParameters) {
	ModbusMaster hmp(241); // Create modbus object that connects to slave id 241 (HMP60)
	hmp.begin(9600); // all nodes must operate at the same speed!
	hmp.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister humidityData(&hmp, 256, true);
	ModbusRegister temperatureData(&hmp, 257, true);

	ModbusMaster co2(240);
	co2.begin(9600);
	ModbusRegister co2Data(&co2, 256, true);

	ModbusRegister co2Status(&co2, 0x800, true);
	ModbusRegister hmpStatus(&hmp, 0x200, true);

	int co2Value = 0;
	int rhValue = 0;
	int tempValue = 0;

	while (true)
	{
		vTaskDelay(5);

		if (hmpStatus.read())
		{
			vTaskDelay(5);
			tempValue = temperatureData.read() / 10;

			vTaskDelay(5);
			rhValue = humidityData.read() / 10;
		}

		vTaskDelay(5);
		if (co2Status.read() == 0)
		{
			vTaskDelay(5);
			co2Value = co2Data.read();
		}
	}
}

static void vLcdUI(void *pvParameters)
{
	(void) pvParameters;

	retarget_init();

	DigitalIoPin sw_a2(1, 8, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a3(0, 5, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a4(0, 6, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a5(0, 7, DigitalIoPin::pullup, true);

	DigitalIoPin rs(0, 29, DigitalIoPin::output);
	DigitalIoPin en(0, 9, DigitalIoPin::output);
	DigitalIoPin d4(0, 10, DigitalIoPin::output);
	DigitalIoPin d5(0, 16, DigitalIoPin::output);
	DigitalIoPin d6(1, 3, DigitalIoPin::output);
	DigitalIoPin d7(0, 0, DigitalIoPin::output);

	LiquidCrystal lcd(&rs, &en, &d4, &d5, &d6, &d7);
	lcd.begin(16, 2);

	int val;

	Menu menu(lcd);

	while(true)
	{
//		if(xQueueReceive(interrupt_q, &val, 5000) == pdTRUE)
//		{
//			Menu::Event event = static_cast <Menu::Event> (val);
//			menu.send(event);
//		}

		//float rh;
		//char buffer[32];

		//vTaskDelay(2000);

		//rh = RH.read()/10.0;
		//snprintf(buffer, 32, "RH=%5.1f%%", rh);
		//printf("%s\n",buffer);
		//lcd->setCursor(0, 1);
		//// Print a message to the LCD.
		//lcd->print(buffer);
	}
}

extern "C" {
  void vStartSimpleMQTTDemo( void ); // ugly - should be in a header
}

int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif

	heap_monitor_setup();

	// initialize RIT (= enable clocking etc.)
	//Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
//	xTaskCreate(task1, "test",
//	configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
//			(TaskHandle_t*) NULL);

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
