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
#include "menu/TextProperty.h"
#include "Disablers.h"

#include "DigitalIoPin.h"
#include "mqtt/connection_info.h"

#define SIGA 0, 5
#define SIGB 0, 6

#define BUTTON_SELECT 1, 8

NumericProperty<int> hum("Humidity", 0, 2000, true, 5);
NumericProperty<int> temp("Temperature", 0, 2000, true, 5);
NumericProperty<int> setPoint("Co2 set", 0, 2000, false, 5);
NumericProperty<int> valve("Valve state", 0, 2000, true, 5);
NumericProperty<int> co2("Co2 value", 0, 2000, true, 5);

//	EEPROM config
static Config config;

//	Menu event queue to which interrupts push events
QueueHandle_t menuEvents = xQueueCreate(10, sizeof(Menu::Event));

/* The following is required if runtime statistics are to be collected
 * Copy the code to the source file where other you initialize hardware */
extern "C" {

//	Sensor data that is set by vMeasure and read by vMqttTask
QueueHandle_t sensorData = xQueueCreate(10, sizeof(int) * 5);

void vConfigureTimerForRunTimeStats(void) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

//	Interrupt handler for the rotary encoder
void PIN_INT0_IRQHandler(void) {
	portBASE_TYPE higherPriorityWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);

	static TickType_t lastTicks = 0;
	TickType_t currentTicks = xTaskGetTickCountFromISR();

	/*	Only send the direction of the rotary encoder when atleast 40 ticks have
	 *	passed since the last interrupt. This is because the rotary encoder might trigger multiple interrupts
	 *	when turned only once. One downside of this is that when the rotary encoder
	 *	is being turned fast enough, we will ignore interrupts that happen too quickly */
	if (currentTicks - lastTicks > 70) {
		//	Determine which way the rotary encode is being turned
		Menu::Event dir = Menu::Event::Up;

		xQueueSendFromISR(menuEvents, (void* )&dir, &higherPriorityWoken);
	}

	lastTicks = currentTicks;
	portEND_SWITCHING_ISR(higherPriorityWoken);
}

//	Interrupt handler for the rotary encoder
void PIN_INT2_IRQHandler(void) {
	portBASE_TYPE higherPriorityWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);

	static TickType_t lastTicks = 0;
	TickType_t currentTicks = xTaskGetTickCountFromISR();

	/*	Only send the direction of the rotary encoder when atleast 40 ticks have
	 *	passed since the last interrupt. This is because the rotary encoder might trigger multiple interrupts
	 *	when turned only once. One downside of this is that when the rotary encoder
	 *	is being turned fast enough, we will ignore interrupts that happen too quickly */
	if (currentTicks - lastTicks > 70) {
		//	Determine which way the rotary encode is being turned
		Menu::Event dir = Menu::Event::Down;

		xQueueSendFromISR(menuEvents, (void* )&dir, &higherPriorityWoken);
	}

	lastTicks = currentTicks;
	portEND_SWITCHING_ISR(higherPriorityWoken);
}

//	Interrupt handler for the confirm button
void PIN_INT1_IRQHandler(void) {
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	portBASE_TYPE higherPriorityWoken = pdFALSE;

	static TickType_t lastTicks = 0;
	TickType_t currentTicks = xTaskGetTickCountFromISR();
	if (currentTicks - lastTicks > 200) {
		Menu::Event event = Menu::Event::Confirm;
		xQueueSendFromISR(menuEvents, (void* )&event, &higherPriorityWoken);
	}
	lastTicks = currentTicks;
	portEND_SWITCHING_ISR(higherPriorityWoken);
}

}
/* end runtime statictics collection */

static void setupGPIOInterrupts(void) {
	/* Initialise PININT driver */
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	/* Set pins back to GPIO and configure as inputs*/
	// Button
	Chip_IOCON_PinMuxSet(LPC_IOCON, BUTTON_SELECT,
			(IOCON_DIGMODE_EN | IOCON_MODE_PULLUP));
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, BUTTON_SELECT);

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
	Chip_INMUX_PinIntSel(2, SIGB); // SIGA
	Chip_INMUX_PinIntSel(1, BUTTON_SELECT);

	/* Configure channel interrupts as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(0));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(0));

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(1));

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(2));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(2));

	/* Enable interrupts in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_SetPriority(PIN_INT0_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(PIN_INT0_IRQn);

	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_SetPriority(PIN_INT1_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(PIN_INT1_IRQn);

	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);
	NVIC_SetPriority(PIN_INT2_IRQn,
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
}

static void idle_delay() {
	vTaskDelay(1);
}

static void vMeasure(void *pvParameters) {
	DigitalIoPin co2_valve(0, 27, DigitalIoPin::output, false);

	ModbusMaster hmpMaster(241); // Create modbus object that connects to slave id 241 (HMP60)
	hmpMaster.begin(9600); // all nodes must operate at the same speed!
	hmpMaster.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister humidityData(&hmpMaster, 256, true);
	ModbusRegister temperatureData(&hmpMaster, 257, true);

	ModbusMaster co2Master(240);
	co2Master.begin(9600);
	ModbusRegister co2Data(&co2Master, 257, true);

	ModbusRegister co2Status(&co2Master, 0x800, true);
	ModbusRegister hmpStatus(&hmpMaster, 0x200, true);

	int co2Value = 0;
	int rhValue = 0;
	int tempValue = 0;

	int offset = 50;
	int data[5] { 0 };

	while (true) {
		vTaskDelay(5);

		if (hmpStatus.read()) {
			tempValue = temperatureData.read() / 10.0;
			temp.setValue(tempValue);
			data[0] = tempValue;
			vTaskDelay(5);

			rhValue = humidityData.read() / 10.0;
			hum.setValue(rhValue);
			data[1] = rhValue;
			DEBUGOUT("temp: %d\r\n", tempValue);
			DEBUGOUT("rh: %d\r\n", rhValue);
		}

		vTaskDelay(5);
		if (co2Status.read() == 0) {
			vTaskDelay(5);
			co2Value = co2Data.read() * 10;

			co2.setValue(co2Value);
			data[2] = co2Value;
			DEBUGOUT("co2: %d\r\n", co2Value);
		}

		if (co2Value <= 0) {
			co2_valve.write(false);
		} else if (co2Value + offset < setPoint.getRealValue()) {
			co2_valve.write(true);
			DEBUGSTR(std::string("Valve on\r\n").c_str());
		} else {
			co2_valve.write(false);
			DEBUGSTR(std::string("Valve off\r\n").c_str());
		}
		vTaskDelay(configTICK_RATE_HZ * 5); //5s
		//DEBUGSTR(std::string("co2: %d\r\n", co2Value).c_str());
		data[3] = co2_valve.read();
		data[4] = setPoint.getRealValue();

		xQueueSend(sensorData, &data, 100);
	}

}

static void vLcdUI(void *pvParameters) {
	(void) pvParameters;

	retarget_init();
	DigitalIoPin rs(0, 29, DigitalIoPin::output);
	DigitalIoPin en(0, 9, DigitalIoPin::output);
	DigitalIoPin d4(0, 10, DigitalIoPin::output);
	DigitalIoPin d5(0, 16, DigitalIoPin::output);
	DigitalIoPin d6(1, 3, DigitalIoPin::output);
	DigitalIoPin d7(0, 0, DigitalIoPin::output);

	LiquidCrystal lcd(&rs, &en, &d4, &d5, &d6, &d7);
	lcd.begin(16, 2);

	Menu::Event event;

	Menu menu(lcd);

	TextProperty ssid("Ssid", "a");
	TextProperty pass("Password", "a");
	TextProperty ip("MQTT IP", "1");
	TextProperty topic("MQTT topic", "a");

	menu.addProperty(setPoint);
	co2.addToMenu(menu);
	hum.addToMenu(menu);
	temp.addToMenu(menu);
	valve.addToMenu(menu);

	menu.addProperty(ssid);
	menu.addProperty(pass);
	menu.addProperty(ip);
	menu.addProperty(topic);

	menu.display();

	while (true) {
		if (xQueueReceive(menuEvents, &event, 5000) == pdTRUE) {
			menu.send(event);
		}
	}
}

extern "C" {
	extern void vMqttTask( void * pvParameters );
}

void vStartMqttTask( void )
{
	static connection_info info;

	info.ssid = "Miko";//config.get("ssid").c_str();
	info.ssidpass = "kissa123";//config.get("ssidpass").c_str();
	info.brokerip = "192.168.43.111";//config.get("brokerip").c_str();
	info.brokerport = 1883;//atoi(config.get("brokerport").c_str());

    xTaskCreate( vMqttTask,          /* Function that implements the task. */
                 "vMqttTask",               /* Text name for the task - only used for debugging. */
				 configMINIMAL_STACK_SIZE * 16, /* Size of stack (in words, not bytes) to allocate for the task. */
                 &info,                     /* Task parameter - not used in this case. */
				 (tskIDLE_PRIORITY + 1UL),         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
                 NULL );                   /* Used to pass out a handle to the created task - not used in this case. */
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
	setupGPIOInterrupts();

	xTaskCreate(vMeasure, "vMeasure",
	configMINIMAL_STACK_SIZE * 6, NULL, (tskIDLE_PRIORITY + 2UL),
			(TaskHandle_t*) NULL);

	xTaskCreate(vLcdUI, "vLcdUI",
	configMINIMAL_STACK_SIZE * 6, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t*) NULL);

	vStartMqttTask();

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
