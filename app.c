/* Bluetooth stack headers */
#include <LIS2DW12.h>
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <math.h>
#include<stdio.h>
#include<stdlib.h>
#include "app.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_rtcc.h"
#include "em_letimer.h"
#include "em_timer.h"

#define PWM_FREQ 50												// PWM frequency

// local function prototypes
void initHardware(void);
void setPWMPulseWidth(TIMER_TypeDef *timer, unsigned int ch, float duration);
int servoSweep(int numServo, float startAngleS, float endAngleS, uint32_t startTimeS , uint32_t endTimeS, uint32_t currentTime);
void delay_ms(uint16_t del);
void delay(int tdelay);
float anglePulse(float widthIn);
float widthPulse(float anglePulse);
void calculateAngle (float x, float y);
float arccos(float n);
// global variables
uint16_t timerTOP;
unsigned int delayTimer = 32768;
float width;
int newDTime;
double eachMSTurn;
uint32_t start, now;
int sweepsNotFinished;
float turnPos;
float widthTurn;
float PI =  3.1415926;
// track position
float pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
float delta1, delta2;
float x ,y;
/* Main application */

void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0											// disable deep sleep
	pconfig->sleep.flags = 0;									// to generate PWM continuously
#endif


	initHardware();
	while(1)
	{

		// calibration motor
		GPIO_PinOutToggle(BSP_LED0_PORT, BSP_LED0_PIN);
		//Leg1
		setPWMPulseWidth(TIMER0, 0, widthPulse(pos1 = 85));
		setPWMPulseWidth(TIMER0, 1, widthPulse(pos2 = 80));
		//Leg2
		setPWMPulseWidth(WTIMER0, 0, widthPulse(pos3 = 90));
		setPWMPulseWidth(WTIMER0, 1, widthPulse(pos4 = 98));
		//Leg3
		setPWMPulseWidth(TIMER1, 0, widthPulse(pos5 = 93));
		setPWMPulseWidth(TIMER1, 1, widthPulse(pos6 = 80));
		//Leg4
		setPWMPulseWidth(TIMER1, 2, widthPulse(pos7 = 87));
		setPWMPulseWidth(TIMER1, 3, widthPulse(pos8 = 95));
		delay(5000);

		//TROATING
		float k = 0;
		while(1)
		{
			k += (PI / 64);
			//LEG 1.
			x = 2 * sin(k) + 2;
			y =  0.25 * cos(k+PI)- 10;
			calculateAngle(x, y);
			setPWMPulseWidth(TIMER0, 0, widthPulse(pos1 = delta1 + 85));
			setPWMPulseWidth(TIMER0, 1, widthPulse(pos2 = 80- delta2 ));
			//LEG2
			float x2 = 2 * sin(k + PI) + 2;
			float y2 =  0.25 * cos(k)- 10;
			calculateAngle(x2, y2);
			setPWMPulseWidth(WTIMER0, 0, widthPulse(pos3 =  90 - delta1));
			setPWMPulseWidth(WTIMER0, 1, widthPulse(pos4 =   delta2 + 98));
			//LEG3
			x = 2 * sin(k+ PI) + 2;
			y = 0.25 * cos(k)- 10;
			calculateAngle(x, y);
			setPWMPulseWidth(TIMER1, 0, widthPulse(pos5 = delta1 + 93));
			setPWMPulseWidth(TIMER1, 1, widthPulse(pos6 = 80 - delta2));
			//LEG4
			x = 2 * sin(k) + 2;
			y = 0.25 * cos(k+PI)- 10;
			calculateAngle(x, y);
			setPWMPulseWidth(TIMER1, 2, widthPulse(pos7 = 87 - delta1));
			setPWMPulseWidth(TIMER1, 3, widthPulse(pos8 = delta2 + 95 ));
			delay_ms(1);


		}





		/*GPIO_PinOutToggle(BSP_LED0_PORT, BSP_LED0_PIN);
		setPWMPulseWidth(TIMER0, 0, widthPulse(pos1 = 0));
		setPWMPulseWidth(TIMER0, 1, widthPulse(pos2 = 0));
		delay(5000);
		start = RTCC_CounterGet();
		do {
			sweepsNotFinished = 0;
			now = RTCC_CounterGet() - start;
			sweepsNotFinished += servoSweep(1, 0, 180, 0, 2000, now);
			sweepsNotFinished += servoSweep(2, 0, 90, 0, 2000, now);
			delay_ms(1);
		}
		while (sweepsNotFinished > 0);*/



		//LIS2DW12_getData();									 get ACC data
		//__NOP();												 put breakpoint here to examine the ACC data

	}

void trotting()
{
	float k = 0;
	while(1)
	{
		//LEG 1.
		k += (PI / 128);
		x = 2 * sin(k) + 2;
		y = 0.5 * cos(k)- 10;
		calculateAngle(x, y);
		setPWMPulseWidth(TIMER0, 0, widthPulse(pos1 = delta1 + 90));
		setPWMPulseWidth(TIMER0, 1, widthPulse(pos2 = 90 - delta2));
		delay_ms(1);

		//LEG 2.

	}
}
}
float arccos(float n)
{
	return acos(n)*180/PI;
}
void calculateAngle (float x, float y)
{
	delta1 = arccos((-x) / sqrt((x * x) + (y * y))) - arccos(((x * x) + (y * y) + 25 - 100) / (10 * sqrt((x * x) + (y * y))));
	delta2 = arccos((x - 4) / sqrt(pow(x - 4, 2) + (y * y))) - arccos(( pow(x-4, 2) + (y * y) + 25 - 100) / (10 * sqrt(pow(x-4, 2) + (y * y))));
}
void delay(int tdelay)
{
	for(int i =0; i < (tdelay / 1000); i++)
	{
		delay_ms(1000);
	}
}

float widthPulse(float angleP)
{
	return (angleP * 0.00733) + 1.0;
}
float anglePulse(float widthIn)
{

	return ((widthIn-1.0) / 0.00733);
}
int servoSweep(int numServo, float startAngleS, float endAngleS, uint32_t startTimeS , uint32_t endTimeS, uint32_t currentTime)
{
	//save servo position
	TIMER_TypeDef *timer;
	int chanelTimer = 0;


	switch(numServo)		//determine which servo
	{
		case 1:
			timer = TIMER0;
			chanelTimer = 0;
			turnPos = pos1;
			break;

		case 2:
			timer = TIMER0;
			chanelTimer = 1;
			turnPos = pos2;
			break;

		case 3:
			timer = WTIMER0;
			chanelTimer = 0;
			turnPos = pos3;
			break;

		case 4:
			timer = WTIMER0;
			chanelTimer = 1;
			turnPos = pos4;
			break;

		case 5:
			timer = TIMER1;
			chanelTimer = 0;
			turnPos = pos5;
			break;

		case 6:
			timer = TIMER1;
			chanelTimer = 1;
			turnPos = pos6;
			break;

		case 7:
			timer = TIMER1;
			chanelTimer = 2;
			turnPos = pos7;
			break;

		case 8:
			timer = TIMER1;
			chanelTimer = 3;
			turnPos = pos8;
			break;
	}

	//Do turn function exception
	if (currentTime < startTimeS) //not in time
	{
	    return 1;
	}
	if (currentTime > endTimeS)		// out of time
	{
	    return 0;
	}

	//Do turn function
	float widthOfPos = (startAngleS * 0.00733); //width of the servo position

	widthTurn = (endAngleS - startAngleS) * 0.00733 - widthOfPos;	//width need to do

	eachMSTurn = widthTurn / (endTimeS - startTimeS); //width in each ms

	widthTurn = widthPulse(turnPos);	// width of new position
	setPWMPulseWidth(timer, chanelTimer , widthTurn += eachMSTurn);

	turnPos = anglePulse(widthTurn);

	switch(numServo)		//return position
	{
		case 1:
			pos1 = turnPos;
			break;

		case 2:

			pos2 = turnPos ;
			break;

		case 3:
			pos3 = turnPos ;
			break;

		case 4:
			pos4 = turnPos;
			break;

		case 5:
			pos5 = turnPos;
			break;

		case 6:
			pos6 = turnPos;
			break;

		case 7:
			pos7 = turnPos;
			break;

		case 8:
			pos8 = turnPos;
			break;
	}
	return 1;

}


void initHardware(void)
{
  	// Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h
	initLog();

	// enable sensor on board and init I2C0 module
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	i2cInit.freq = 400000;
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_RTCC, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(gpioPortC, 11, gpioModeInputPull, 1);		// sensor INT, pull-up
	GPIO_PinModeSet(gpioPortC, 9, gpioModeWiredAnd, 1);			// sensor SDA
	GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 1);		// sensor SCL
	I2C0->ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC14 |
  			  	  	  I2C_ROUTELOC0_SCLLOC_LOC14;
	I2C0->ROUTEPEN = 3;
	I2C_Init(I2C0, &i2cInit);
	LIS2DW12_setup();											// init accelerometer

	// switch pins configuration
	GPIO_PinModeSet(gpioPortD, 15, gpioModeInputPull, 1);		// leg switch pull-up
	GPIO_PinModeSet(gpioPortA, 0, gpioModeInputPull, 1);		// leg switch pull-up
	GPIO_PinModeSet(gpioPortF, 5, gpioModeInputPull, 1);		// leg switch pull-up
	GPIO_PinModeSet(gpioPortF, 6, gpioModeInputPull, 1);		// leg switch pull-up

	// init LED
	GPIO_PinModeSet(BSP_LED0_PORT, BSP_LED0_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, 0);

	// init PWM pins
	GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortA, 3, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortA, 4, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortF, 2, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortF, 3, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortF, 5, gpioModePushPull, 0);

	// LETIMER setup
	LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
	CMU_ClockEnable(cmuClock_LETIMER0, true);
	letimerInit.enable = false;
	letimerInit.comp0Top  = true;
	letimerInit.topValue = delayTimer; //32768 = 1s
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);			// enable interrupt
	LETIMER_Init(LETIMER0, &letimerInit);
	//LETIMER0->CMD = LETIMER_CMD_STOP;
	//NVIC_ClearPendingIRQ(LETIMER0_IRQn);
	//NVIC_EnableIRQ(LETIMER0_IRQn);
	//LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);

	// PWM setup
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockEnable(cmuClock_WTIMER0, true);
	// Configure Compare/Capture for output compare
	// Use PWM mode, which sets output on overflow and clears on compare events
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModePWM;
	TIMER_InitCC(TIMER0, 0, &timerCCInit);
	TIMER_InitCC(TIMER0, 1, &timerCCInit);
	TIMER_InitCC(TIMER1, 0, &timerCCInit);
	TIMER_InitCC(TIMER1, 1, &timerCCInit);
	TIMER_InitCC(TIMER1, 2, &timerCCInit);
	TIMER_InitCC(TIMER1, 3, &timerCCInit);
	TIMER_InitCC(WTIMER0, 0, &timerCCInit);
	TIMER_InitCC(WTIMER0, 1, &timerCCInit);
	TIMER0->ROUTELOC0 |=  TIMER_ROUTELOC0_CC0LOC_LOC1 +
			              TIMER_ROUTELOC0_CC1LOC_LOC1;
	TIMER0->ROUTEPEN  |=  TIMER_ROUTEPEN_CC0PEN +
						  TIMER_ROUTEPEN_CC1PEN;
	TIMER1->ROUTELOC0 |=  TIMER_ROUTELOC0_CC0LOC_LOC29 +
				          TIMER_ROUTELOC0_CC1LOC_LOC27 +
						  TIMER_ROUTELOC0_CC2LOC_LOC25 +
						  TIMER_ROUTELOC0_CC3LOC_LOC23;
	TIMER1->ROUTEPEN  |=  TIMER_ROUTEPEN_CC0PEN +
						  TIMER_ROUTEPEN_CC1PEN +
						  TIMER_ROUTEPEN_CC2PEN +
						  TIMER_ROUTEPEN_CC3PEN;
	WTIMER0->ROUTELOC0 |= WTIMER_ROUTELOC0_CC0LOC_LOC3 +
				          WTIMER_ROUTELOC0_CC1LOC_LOC2;
	WTIMER0->ROUTEPEN  |= WTIMER_ROUTEPEN_CC0PEN +
						  WTIMER_ROUTEPEN_CC1PEN;
	timerTOP = (CMU_ClockFreqGet(cmuClock_TIMER0) >> 4) / PWM_FREQ;
	TIMER_TopSet(TIMER0, timerTOP); // set PWM freq
	TIMER_TopSet(TIMER1, timerTOP);
	TIMER_TopSet(WTIMER0, timerTOP);
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
	timerInit.prescale = timerPrescale16;
	TIMER_Init(TIMER0, &timerInit);
	TIMER_Init(TIMER1, &timerInit);
	TIMER_Init(WTIMER0, &timerInit);


// 	gecko_init(pconfig);									// Initialize Bluetooth stack
}
void delay_ms(uint16_t del)
{
	LETIMER0->COMP0 = (int)(del*32.768);
	LETIMER_Enable(LETIMER0, true);
	while (!(LETIMER0->IF & LETIMER_IF_UF)){}
	LETIMER_Enable(LETIMER0, false);
	LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);

}

void setPWMPulseWidth(TIMER_TypeDef *timer, unsigned int ch, float width) // pulse width in ms
{
	timer->CC[ch].CCV = (width*timerTOP*PWM_FREQ)/1000;
}

void LETIMER0_IRQHandler(void)
{
	LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
}
