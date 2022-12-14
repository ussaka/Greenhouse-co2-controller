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

<<<<<<< HEAD
#include "DigitalIoPin.h"
#include <vector>

#define SIGA 0, 5
#define SIGB 0, 6

#define BUTTON_SELECT 1, 8

//	EEPROM config
static Config config;

//	Menu event queue to which interrupts push events
QueueHandle_t menuEvents = xQueueCreate(10, sizeof(Menu::Event));
=======
// TODO: insert other definitions and declarations here
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9

//	Sensor value queue. Contains Co2, Temperature, humidity and solenoid position
QueueHandle_t data_q = xQueueCreate(10, sizeof(int) * 4);

/* The following is required if runtime statistics are to be collected
 * Copy the code to the source file where other you initialize hardware */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

<<<<<<< HEAD
//	Interrupt handler for the rotary encoder
void PIN_INT0_IRQHandler(void)
{
	portBASE_TYPE higherPriorityWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);

	static TickType_t lastTicks = 0;
	TickType_t currentTicks = xTaskGetTickCountFromISR();

	/*	Only send the direction of the rotary encoder when atleast 40 ticks have
	 *	passed since the last interrupt. This is because the rotary encoder might trigger multiple interrupts
	 *	when turned only once. One downside of this is that when the rotary encoder
	 *	is being turned fast enough, we will ignore interrupts that happen too quickly */
	if(currentTicks - lastTicks > 40)
	{
		//	Determine which way the rotary encode is being turned
		Menu::Event dir = Chip_GPIO_GetPinState(LPC_GPIO, SIGA) ? Menu::Event::Up : Menu::Event::Down;
		xQueueSendFromISR(menuEvents, (void*)&dir, &higherPriorityWoken);
	}

	lastTicks = currentTicks;
	portEND_SWITCHING_ISR(higherPriorityWoken);
}

//	Interrupt handler for the confirm button
void PIN_INT1_IRQHandler(void)
{
	portBASE_TYPE higherPriorityWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);

	Menu::Event event = Menu::Event::Confirm;
	xQueueSendFromISR(menuEvents, (void*)&event, &higherPriorityWoken);

	portEND_SWITCHING_ISR(higherPriorityWoken);
}

=======
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9
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

static void vSendMQTT(void *pvParameters) {
	while (1) {
		vTaskDelay(50);
	}
}

static void vMeasure(void *pvParameters) {
<<<<<<< HEAD
	std::vector <int> data;

	DigitalIoPin co2_valve(0, 27, DigitalIoPin::output, false);

=======
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9
	ModbusMaster hmp(241); // Create modbus object that connects to slave id 241 (HMP60)
	hmp.begin(9600); // all nodes must operate at the same speed!
	hmp.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister humidityData(&hmp, 256, true);
	ModbusRegister temperatureData(&hmp, 257, true);

	ModbusMaster co2(240);
	co2.begin(9600);
	ModbusRegister co2Data(&co2, 257, true);

	ModbusRegister co2Status(&co2, 0x800, true);
	ModbusRegister hmpStatus(&hmp, 0x200, true);

	int co2Value = 0;
	int rhValue = 0;
	int tempValue = 0;
<<<<<<< HEAD
	int set_point = 750;
	int offset = 20;
=======
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9

	while (true)
	{
		vTaskDelay(5);

		if (hmpStatus.read())
		{
<<<<<<< HEAD
			tempValue = temperatureData.read() / 10.0;
			data.push_back(tempValue);
=======
			vTaskDelay(5);
			tempValue = temperatureData.read() / 10;

>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9
			vTaskDelay(5);
			rhValue = humidityData.read() / 10.0;
			data.push_back(rhValue);
		}

		vTaskDelay(5);
		if (co2Status.read() == 0)
		{
			vTaskDelay(5);
<<<<<<< HEAD
			co2Value = co2Data.read() / 10.0;
			if (co2Value + offset < set_point) {
				data.push_back(co2Value);
				co2_valve.write(true);
				DEBUGSTR(std::string("Valve on\r\n").c_str());
			}
		}
		vTaskDelay(configTICK_RATE_HZ * 5); //5s
		co2_valve.write(false);
		data.push_back(co2_valve.read());
		xQueueSend(data_q, &data, 100);
		DEBUGOUT("co2: %d\r\n", co2Value);
		//DEBUGSTR(std::string("co2: %d\r\n", co2Value).c_str());
		DEBUGSTR(std::string("Valve off\r\n").c_str());
=======
			co2Value = co2Data.read();
		}
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9
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

<<<<<<< HEAD
	Menu::Event event;
=======
	int val;

>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9
	Menu menu(lcd);

	NumericProperty <int> setPoint("setpoint", 0, 2000, false, 5);
	menu.addProperty(setPoint);

	menu.display();

	while(true)
	{
<<<<<<< HEAD
		if(xQueueReceive(menuEvents, &event, 5000) == pdTRUE)
		{
			menu.send(event);
		}
=======
//		if(xQueueReceive(interrupt_q, &val, 5000) == pdTRUE)
//		{
//			Menu::Event event = static_cast <Menu::Event> (val);
//			menu.send(event);
//		}
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9

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
  void vStartMqttTask( void ); // ugly - should be in a header
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
<<<<<<< HEAD
	config.read();

	if(!config.exists("ssid"))
		config.set("ssid", "none");

	if(!config.exists("ssidpass"))
		config.set("ssidpass", "none");

	if(!config.exists("brokerip"))
		config.set("brokerip", "none");

	if(!config.exists("setpoint"))
		config.set("setpoint", "0");
=======
>>>>>>> 0c8303b5109e303d24cb7a3e14a735c77db200b9

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

	vStartMqttTask();
	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
