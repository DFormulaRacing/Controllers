//Microcontroller code
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_can.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"

float slip;  //Global variables for launch control
uint32_t Torque;
int count, flag, TimeCount = 0;
uint32_t old_torque, sent_torque = 0;
float old_slip = 0;

uint32_t g_ui32Flags;   // Global variables for CAN
#define PRINT_BUFF_SIZE 8
char g_pcPrintBuff[PRINT_BUFF_SIZE];

volatile uint32_t g_ui32Msg1Count = 0;
volatile uint32_t g_ui32Msg2Count = 0;

volatile bool g_bMsgObj3Sent = 0;

tCANMsgObject g_sCANMsgObject1;
tCANMsgObject g_sCANMsgObject2;
tCANMsgObject g_sCANMsgObject3;

uint8_t g_pui8Msg1[4] = { 0, 0, 0, 0 };
uint8_t g_pui8Msg2[5] = { 2, 2, 2, 2, 2 };

// Flags for the interrupt handler to indicate that a message was received.
volatile bool g_bRXFlag1 = 0;
volatile bool g_bRXFlag2 = 0;
volatile bool g_bRXFlag3 = 0;
volatile bool g_bRXFlag4 = 0;
volatile bool g_bRXFlag5 = 0;

// A flag to indicate that some reception error occurred.
volatile bool g_bErrFlag = 0;

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// Configure the UART and its pins for debugging.
void
ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}
void LED_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //enable ports
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3  //configure pins as GPIO
| GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 
					|GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //unlock PD7
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;   
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //unlock PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_AFSEL) &= ~0x01;   
	HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0); //initialize to zero
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);
}
void Input_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);//enable port
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1); //configure pins
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
}	
void CAN_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  //enable port
	GPIOPinConfigure(GPIO_PE4_CAN0RX); //configure pins as CAN
  	GPIOPinConfigure(GPIO_PE5_CAN0TX);
	GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5); //set mux for CAN
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0); //enable CAN 
  	CANInit(CAN0_BASE); //initialize CAN base
}
void PWM_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); //enable port
	// Set the PWM clock to the system clock.
 	 SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	// The PWM peripheral must be enabled for use.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	// This is necessary if your part supports GPIO pin function muxing.
	GPIOPinConfigure(GPIO_PN2_M0PWM6);
	// Configure the PWM function for this pin.
	GPIOPinTypePWM(GPIO_PORTN_BASE, GPIO_PIN_2);
}
void Systick_init(void)
{
	ROM_SysTickPeriodSet(SysCtlClockGet()/1000); //set SysTick to
	ROM_IntMasterEnable();											 //interrupt every ms
	ROM_SysTickIntEnable();
}

void Systick_start(void)
{
	ROM_SysTickEnable(); //start SysTick
}

void Timer_init (void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); //enable timers
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); //enable GPIO bank
	//mux pins for timer mode
	ROM_GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PL2_T1CCP0);
	ROM_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PL3_T1CCP1);
	ROM_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PL4_T2CCP0);
	ROM_GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PL5_T2CCP1);
	//configure timer
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet());
	// Set the pin to use the internal pull-up.
	ROM_GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_2,
					 GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	ROM_GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_3,
					 GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	ROM_GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_4,
					 GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	ROM_GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_5,
					 GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	// Enable processor interrupts.
	ROM_IntMasterEnable();
	// Configure the timers in downward edge count mode.
	ROM_TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR |
					 TIMER_CFG_A_CAP_COUNT));
	ROM_TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR |
					 TIMER_CFG_B_CAP_COUNT));
	ROM_TimerConfigure(TIMER2_BASE, (TIMER_CFG_SPLIT_PAIR |
					 TIMER_CFG_A_CAP_COUNT));
	ROM_TimerConfigure(TIMER2_BASE, (TIMER_CFG_SPLIT_PAIR |
					 TIMER_CFG_B_CAP_COUNT));
	//Set timers
	ROM_TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	ROM_TimerControlEvent(TIMER1_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
	ROM_TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	ROM_TimerControlEvent(TIMER2_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
	ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 1000);
	ROM_TimerLoadSet(TIMER1_BASE, TIMER_B, 1000);
	ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, 1000);
	ROM_TimerLoadSet(TIMER2_BASE, TIMER_B, 1000);
	// Setup the interrupt.
	ROM_IntEnable(INT_TIMER0B);
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
}
void Timer_start(void) //start all timers for launch control
{
	ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	ROM_TimerEnable(TIMER1_BASE, TIMER_B);
	ROM_TimerEnable(TIMER2_BASE, TIMER_A);
	ROM_TimerEnable(TIMER2_BASE, TIMER_B);
	ROM_TimerEnable(TIMER0_BASE, TIMER_B);
}
void //print CAN for debugging
PrintCANMessageInfo(tCANMsgObject *psCANMsg, uint32_t ui32MsgObj)
{
	unsigned int uIdx;
	// Check to see if there is an indication that some messages were
	// lost.
	if(psCANMsg->ui32Flags & MSG_OBJ_DATA_LOST)
	{
		UARTprintf("CAN message loss detected on message object %d\n",  ui32MsgObj);
	}
	// Print out the contents of the message that was received.
	UARTprintf("Msg Obj=%u ID=0x%05X len=%u data=0x", ui32MsgObj,
						 psCANMsg->ui32MsgID, psCANMsg->ui32MsgLen);
	for(uIdx = 0; uIdx < psCANMsg->ui32MsgLen; uIdx++)
	{
		UARTprintf("%02X ", psCANMsg->pui8MsgData[uIdx]);
	}
	UARTprintf("\n");
}
void PWM_start(void)
{
	// Configure the PWM0 to count up/down without synchronization.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
				PWM_GEN_MODE_NO_SYNC);
	// Set the PWM period to 250Hz.
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 64000);
	// Set PWM0 to a duty cycle of 50%.
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 32000);
	// Enable the PWM0 Bit6 (PL2) output signal.
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	// Enable the PWM generator block.
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}
void
CANIntHandler(void)
{
	uint32_t ui32Status;
	// Read the CAN interrupt status to find the cause of the interrupt
	ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
	// If the cause is a controller status interrupt, then get the status
	if(ui32Status == CAN_INT_INTID_STATUS)
	{
		// Read the controller status.
		ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
		// Set a flag to indicate some errors may have occurred.
		g_bErrFlag = 1;
	}
	// Check if the cause is message object 1.
	else if(ui32Status == 1)
	{
		//Clear the message object interrupt.
		CANIntClear(CAN0_BASE, 1);
		// Set flag to indicate received message is pending for this message  object.
		g_bRXFlag1 = 1;
		// Since a message was received, clear any error flags.
		g_bErrFlag = 0;
	}
	// Check if the cause is message object 2.
	else if(ui32Status == 2)
	{
		CANIntClear(CAN0_BASE, 2);
		g_bRXFlag2 = 1;
		g_bErrFlag = 0;
	}
	// Check if the cause is message object 3.
	else if(ui32Status == 3)
	{
		CANIntClear(CAN0_BASE, 3);
		g_bRXFlag3 = 1;
		g_bErrFlag = 0;
	}
	// Check if the cause is message object 4.
	else if(ui32Status == 4)
	{
		CANIntClear(CAN0_BASE, 4);
		g_bRXFlag4 = 1;
		g_bErrFlag = 0;
	}
	// Check if the cause is message object 5.
	else if(ui32Status == 5)
	{
		CANIntClear(CAN0_BASE, 5);
		g_bRXFlag5 = 1;
		g_bErrFlag = 0;
	}
	// Check if the cause is message object 6.
	else if(ui32Status == 6)
	{
		CANIntClear(CAN0_BASE, 6);
		g_ui32Msg1Count++;
		g_bErrFlag = 0;
	}
	else
	{
		UARTprintf("Error");
	}
}
void CAN_start(void)
{
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
	// Enable interrupts on the CAN peripheral. 
	CANIntRegister(CAN0_BASE, CANIntHandler);
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	// Enable the CAN interrupt on the processor (NVIC).
	IntEnable(INT_CAN0);
	// Enable the CAN for operation.
	CANEnable(CAN0_BASE);
}
void
TimerIntHandler(void)
{
	int TimeVal1,TimeVal2,TimeVal3,TimeVal4;	
	int RAvg, FAvg;
	TimeCount++;
	if (TimeCount >= 100000)
	{
		//clear interrupt
		ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
		//get count from hall effects
		TimeVal1 = ROM_TimerValueGet(TIMER1_BASE, TIMER_A);
		TimeVal2 = ROM_TimerValueGet(TIMER1_BASE, TIMER_B);
		TimeVal3 = ROM_TimerValueGet(TIMER2_BASE, TIMER_A);
		TimeVal4 = ROM_TimerValueGet(TIMER2_BASE, TIMER_B);
		//disable timers
		ROM_TimerDisable(TIMER1_BASE, TIMER_A);
		ROM_TimerDisable(TIMER1_BASE, TIMER_B);
		ROM_TimerDisable(TIMER2_BASE, TIMER_A);
		ROM_TimerDisable(TIMER2_BASE, TIMER_B);
		//reload timers
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 1000);
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_B, 1000);
		ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, 1000);
		ROM_TimerLoadSet(TIMER2_BASE, TIMER_B, 1000);
		//calculate number of rising edges
		TimeVal1 = 1000 - TimeVal1;
		TimeVal2 = 1000 - TimeVal2;
		TimeVal3 = 1000 - TimeVal3;
		TimeVal4 = 1000 - TimeVal4;
		//calculate front and rear average
		RAvg = (int)(TimeVal1 + TimeVal2)/2;
		FAvg = (int)(TimeVal3 + TimeVal4)/2;
		//calculate slip
		slip = (RAvg - FAvg)/RAvg;
		//enable timers
		ROM_TimerEnable(TIMER1_BASE, TIMER_A);
		ROM_TimerEnable(TIMER1_BASE, TIMER_B);
		ROM_TimerEnable(TIMER2_BASE, TIMER_A);
		ROM_TimerEnable(TIMER2_BASE, TIMER_B);
		//reset count
		TimeCount = 0;
	}
}

void SimpleDelay(void)
{
    // Delay cycles for 1 second
    SysCtlDelay(16000000 / 3);
}

void
SysTickIntHandler(void)
{
	uint8_t msg[2];
	float slope;
	//calculate slope
	slope = (Torque - old_torque)/(slip - old_slip);
	old_torque = Torque;
	old_slip = slip;
	if (slope < 0 )
	{
		count++;
		if (count > 10) //eliminate noise
		{
			if (flag == 0)
			{
				flag = 1;
				Torque = (uint32_t)(.75*Torque);
				msg[1] = (uint8_t)(0x0000000F & Torque);
				msg[0] = (uint8_t)(Torque >> 8);
				sent_torque = Torque;
			}else
			{ //keep lowering torque
				Torque = (uint32_t)(.75*sent_torque);
				msg[1] = (uint8_t)(0x0000000F & Torque);
				msg[0] = (uint8_t)(Torque >> 8);
				sent_torque = Torque;
			}
		}else	
		{	//send unmodified torque
			msg[1] = (uint8_t)(0x0000000F & Torque);
			msg[0] = (uint8_t)(Torque >> 8);
			sent_torque = Torque;
			flag = 0;
		}
	}else
	{ //send unmodified torque
		msg[1] = (uint8_t)(0x0000000F & Torque);
		msg[0] = (uint8_t)(Torque >> 8);
		sent_torque = Torque;
		count = 0;
		flag = 0;
	}
	//send torque over CAN
	g_sCANMsgObject1.pui8MsgData = msg;
	CANMessageSet(CAN0_BASE, 6, &g_sCANMsgObject1, MSG_OBJ_TYPE_TX);
	PrintCANMessageInfo(&g_sCANMsgObject1, 6);

	SimpleDelay();
	// Check the error flag to see if errors occurred
	if(g_bErrFlag)
	{
			UARTprintf(" error - cable connected?\n");
	}
}

int main(void)
{	
		uint32_t Data, Data1, Data2; //For CAN decoding
		
		tCANMsgObject rCANMessage; //CAN variables
		uint8_t pui8Msg1Data[8];
		uint8_t pui8Msg2Data[8];
		uint8_t pui8Msg3Data[8];
		uint8_t pui8Msg4Data[8];
		uint8_t pui8Msg5Data[8];
		g_sCANMsgObject1.ui32MsgID = 0x300;//TX Throttle
    g_sCANMsgObject1.ui32MsgIDMask = 0;
    g_sCANMsgObject1.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCANMsgObject1.ui32MsgLen = sizeof(2);
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    // Initialize the UART.
    ConfigureUART();
		LED_init();
		CAN_init();
		CAN_start();
    // Initialize a message object to be used for receiving CAN messages with
    // any CAN ID.
    rCANMessage.ui32MsgID = 0x710; //Motor Temps
    rCANMessage.ui32MsgIDMask = 0xFFFF;
    rCANMessage.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
    rCANMessage.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, 1, &rCANMessage, MSG_OBJ_TYPE_RX);
    rCANMessage.ui32MsgID = 0x626; //State of Charge
    rCANMessage.ui32MsgLen = 5;
    CANMessageSet(CAN0_BASE, 2, &rCANMessage, MSG_OBJ_TYPE_RX);
    rCANMessage.ui32MsgID = 0x627; //Battery Temp
    rCANMessage.ui32MsgLen = 6;
    CANMessageSet(CAN0_BASE, 3, &rCANMessage, MSG_OBJ_TYPE_RX); 
    rCANMessage.ui32MsgID = 0x700; //Throttle in
    rCANMessage.ui32MsgLen = 2;
    CANMessageSet(CAN0_BASE, 4, &rCANMessage, MSG_OBJ_TYPE_RX); 
    rCANMessage.ui32MsgID = 0x712; //Precharge state
    rCANMessage.ui32MsgLen = 2;
    CANMessageSet(CAN0_BASE, 5, &rCANMessage, MSG_OBJ_TYPE_RX); 
		
		PWM_init();
		PWM_start();
		Timer_init();
		Timer_start();
		Systick_init();
		Systick_start();
		
//CAN
//RX
    for(;;)
    {
        // If the flag is set, that means that the RX interrupt occurred and
        // there is a message ready to be read from the CAN
        if(g_bRXFlag1) //temps from MC
        {
            rCANMessage.pui8MsgData = pui8Msg1Data;
            // Read the message from the CAN.  
            CANMessageGet(CAN0_BASE, 1, &rCANMessage, 0);
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            g_bRXFlag1 = 0;
            //Print information about the message just received.
            //PrintCANMessageInfo(&rCANMessage, 1);
           //Decode data
	Data = (uint32_t)((pui8Msg1Data[1]*256)+pui8Msg1Data[0])/10 - 20;
	//UARTprintf("Motor Temp = %d\n",Data);
	if (Data > 140)
	{ //turn on warning light
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0xFF);
	}
	else if (Data < 135)
	{ //Turn off warning light with hysteresis
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0);
	}	
	Data = (uint32_t)((pui8Msg1Data[3] *256)+pui8Msg1Data[2])/10 - 20;
	//UARTprintf("Logic Board Temp = %d\n",Data);
	Data1 = (uint32_t)((pui8Msg1Data[5] *256)+pui8Msg1Data[4])/10 - 20;
	//UARTprintf("Driver Board Temp = %d\n",Data);
	Data2 = (uint32_t)((pui8Msg1Data[7] *256)+pui8Msg1Data[6])/10 - 20;
//UARTprintf("IGBT Board Temp = %d\n",Data);
	if ((Data > 90) || (Data1 > 90) || (Data2 > 90))
	{ //Turn on warning light
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, 0xFF);
	}
	else if ((Data < 85) && (Data1 < 85) && (Data2 < 85))
	{ //Turn off warning light with hysteresis
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_5, 0);
	}
        }
        if(g_bRXFlag2) //state of charge
        {
            rCANMessage.pui8MsgData = pui8Msg2Data;
            CANMessageGet(CAN0_BASE, 2, &rCANMessage, 0);
            g_bRXFlag2 = 0;
            //PrintCANMessageInfo(&rCANMessage, 2);
	if (pui8Msg2Data[0] < 13)
	{ //Control SOC display
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else if (pui8Msg2Data[0] < 26)	
	{		
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else if (pui8Msg2Data[0] < 39)
	{		
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else if (pui8Msg2Data[0] < 52)	
	{		
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else if (pui8Msg2Data[0] < 65)	
	{		
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else if (pui8Msg2Data[0] < 78)	
	{		
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else if (pui8Msg2Data[0] < 91)	
	{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0);	
	} else
	{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_5, 0xFF);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_6, 0xFF);	
	} 
}
if(g_bRXFlag3) //Temps from Batt
{
 rCANMessage.pui8MsgData = pui8Msg3Data;
 CANMessageGet(CAN0_BASE, 3, &rCANMessage, 0);
 g_bRXFlag3 = 0;
 //PrintCANMessageInfo(&rCANMessage, 3);
if (pui8Msg3Data[4] > 50)
{ //Turn on warning light
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0xFF);
}
else if (pui8Msg3Data[4] < 45)
{ //Turn off warning light with hysteresis
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
}
}
if(g_bRXFlag4) //throttle
{	
rCANMessage.pui8MsgData = pui8Msg4Data;
	CANMessageGet(CAN0_BASE, 4, &rCANMessage, 0);
	g_bRXFlag4 = 0;
	//PrintCANMessageInfo(&rCANMessage, 4);
	//Store requested torque
	Torque = (uint32_t)((pui8Msg4Data[1]*256)+pui8Msg4Data[0]);
}
if(g_bRXFlag5) //Precharge state
{
	rCANMessage.pui8MsgData = pui8Msg5Data;
	CANMessageGet(CAN0_BASE, 5, &rCANMessage, 0);
	g_bRXFlag5 = 0;
	//PrintCANMessageInfo(&rCANMessage, 5);
	//UARTprintf("State = %d\n",pui8Msg5Data[1]);
	if (pui8Msg5Data[1] == 0)
{ //Control precharge lights
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
}
else if (pui8Msg5Data[1] == 1)
{
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF);
}	
else
{
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
}
}   
}
}
