/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "MainTask.h"
#include "cmsis_os.h"
#include "Priorities.h"
#include "ProcessorInit.h"

/* Include Periphiral drivers */
//#include "ADC.h"
//#include "EEPROM.h"
#include "Encoder.h"
//#include "I2C.h"
#include "InputCapture.h"
#include "IO.h"
#include "PWM.h"
#include "RCReceiver.h"
#include "Servo.h"
//#include "SMBus.h"
//#include "SPI.h"
#include "Timer.h"
//#include "UART.h"
#include "USBCDC.h"
//#include "Watchdog.h"

/* Include Device drivers */
//#include "Battery.h"
//#include "ESCON.h"
//#include "IMU.h"
//#include "LCD.h"
#include "LSPC.hpp"
//#include "MPU9250.h"
//#include "MTI200.h"
//#include "QuadratureKnob.h"

/* Include Module libraries */
//#include "LQR.h"
//#include "SlidingMode.h"
#include "Debug.h"
//#include "COMEKF.h"
//#include "MadgwickAHRS.h"
//#include "QEKF.h"
//#include "VelocityEKF.h"
//#include "Parameters.h"
//#include "PowerManagement.h"
//#include "Joystick.h"
//#include "ModuleTemplate.h"
#include "SpeedController.h"

/* Include Application-layer libraries */
#include "LightAndSoundHandler.h"
//#include "Communication.h"
//#include "FrontPanel.h"
//#include "PathFollowingController.h"
//#include "ApplicationTemplate.h"

/* Miscellaneous includes */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <string>
#include <vector>

void jetColor(const float v, const float vmin, const float vmax, float RGB[3]);
void Reboot_Callback(void * param, const std::vector<uint8_t>& payload);
void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload);

int32_t encoderFront = 0;
int32_t encoderBack = 0;

float throttle = 0;
float steering = 0;

void MainTask(void * pvParameters)
{
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */

	/* Initialize communication */
	USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY);
	LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	Debug::AssignDebugCOM(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality

	/* Initialize hardware periphirals */
	RCReceiver * rc_throttle = new RCReceiver(InputCapture::TIMER4, InputCapture::CH3);
	RCReceiver * rc_steering = new RCReceiver(InputCapture::TIMER4, InputCapture::CH4);
	Servo * throttle = new Servo(PWM::TIMER1, PWM::CH1);
	Servo * servo_rear = new Servo(PWM::TIMER9, PWM::CH1);
	Encoder * encoder_front = new Encoder(Encoder::TIMER5);
	Encoder * encoder_back = new Encoder(Encoder::TIMER2, true);
	IO * buzzer = new IO(GPIOA, GPIO_PIN_4);
	PWM * red = new PWM(PWM::TIMER8, PWM::CH1, 1000, 100);
	PWM * green = new PWM(PWM::TIMER8, PWM::CH2);
	PWM * blue = new PWM(PWM::TIMER8, PWM::CH3);

	/* Register general (system wide) LSPC callbacks */
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::Reboot, &Reboot_Callback);
	lspcUSB->registerCallback(lspc::MessageTypesFromPC::EnterBootloader, &EnterBootloader_Callback);

	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER11, 1000000); // create a 1 MHz counting timer used for micros() timing

	/* Initialize modules */
	SpeedController * controller = new SpeedController(lspcUSB, *microsTimer, *throttle, *encoder_front, SPEED_CONTROLLER_PRIORITY);
	controller->Enable();
	controller->SetSpeed(0.5);

	/******* APPLICATION LAYERS *******/
	LightAndSoundHandler * LightAndSound = new LightAndSoundHandler(*red, *green, *blue, *buzzer);
	if (!LightAndSound) ERROR("Could not initialize light and sound handler");


	/* Send CPU load every second */
	char * pcWriteBuffer = (char *)pvPortMalloc(1024);
	while (1)
	{
		vTaskGetRunTimeStats(pcWriteBuffer);
		char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
		*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

		// Split into multiple packages and send
		uint16_t txIdx = 0;
		uint16_t remainingLength = strlen(pcWriteBuffer);
		uint16_t txLength;

		while (remainingLength > 0) {
			txLength = remainingLength;
			if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
				txLength = LSPC_MAXIMUM_PACKAGE_LENGTH-1;
				while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
				if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
				else txLength++;
			}
			lspcUSB->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

			txIdx += txLength;
			remainingLength -= txLength;
		}
		osDelay(1000);
	}

	while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}
}

void Reboot_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	NVIC_SystemReset();
}

void EnterBootloader_Callback(void * param, const std::vector<uint8_t>& payload)
{
	// ToDo: Need to check for magic key
	USBD_Stop(&USBCDC::hUsbDeviceFS);
	USBD_DeInit(&USBCDC::hUsbDeviceFS);
	Enter_DFU_Bootloader();
}
