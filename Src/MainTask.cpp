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
#include "CPULoad.h"
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

float throttle_in = 0;
float steering_in = 0;

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
	USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY, "JetsonCar Serial Port", 0x5544);
	LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	Debug::AssignDebugCOM(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality
	CPULoad * cpuLoad = new CPULoad(*lspcUSB, CPULOAD_PRIORITY);

	/* Initialize hardware periphirals */
	RCReceiver * rc_throttle = new RCReceiver(InputCapture::TIMER4, InputCapture::CH3);
	RCReceiver * rc_steering = new RCReceiver(InputCapture::TIMER4, InputCapture::CH4);
	Servo * throttle = new Servo(PWM::TIMER1, PWM::CH1);
	Servo * servo_front = new Servo(PWM::TIMER9, PWM::CH1);
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
	/*SpeedController * controller = new SpeedController(lspcUSB, *microsTimer, *throttle, *encoder_back, 480, SPEED_CONTROLLER_PRIORITY);  // 12 ticks pr. encoder/motor rev,  gearing ratio of 40  =  480 ticks pr. rev
	controller->Enable();
	controller->SetSpeed(5);
	servo_front->Set(0.0f);*/

	throttle->Disable();
	osDelay(100);
	throttle->Set(0);
	osDelay(100);
	throttle->Disable();
	osDelay(100);
	throttle->Set(0);
	osDelay(1000);


	/******* APPLICATION LAYERS *******/
	LightAndSoundHandler * LightAndSound = new LightAndSoundHandler(*red, *green, *blue, *buzzer);
	if (!LightAndSound) ERROR("Could not initialize light and sound handler");


	bool EnableTest = false;
	float SpeedValue = 0.0f;
	bool Up_nDown = true;
	uint16_t stepWait = 0;

	while (1)
	{
		encoderFront = encoder_front->Get();
		encoderBack = encoder_back->Get();

		throttle_in = rc_throttle->Get();
		steering_in = rc_steering->Get();

		if (steering_in < -0.7)
			EnableTest = true;
		else if (steering_in > 0.7) {
			EnableTest = false;
			SpeedValue = 0;
		}

		if (EnableTest) {
			stepWait++;

			if ((stepWait % 40) == 0) { // 40x50 = 2000 ms
				stepWait = 0;

				if (Up_nDown) {
					SpeedValue += 0.05f;
					if (SpeedValue >= 0.79f)
						Up_nDown = false;
				}
				else
				{
					SpeedValue -= 0.05f;
					if (SpeedValue <= -0.79f)
						Up_nDown = true;
				}
			}
		} else {
			SpeedValue = throttle_in;
		}

		throttle->Set(SpeedValue);
		//controller->SetSpeed(10*throttle_in);
		servo_front->Set(steering_in);


		lspc::MessageTypesToPC::Sensors_t msg;
		msg.timestamp = microsTimer->GetTime();
		msg.encoders.front = encoderFront;
		msg.encoders.back = encoderBack;
		msg.rc.throttle = throttle_in;
		msg.rc.steering = steering_in;
		msg.motors.throttle = float(int16_t(1024*SpeedValue)) / 1024.f;
		msg.motors.steering = float(int16_t(1024*steering_in)) / 1024.f;
		lspcUSB->TransmitAsync(lspc::MessageTypesToPC::Sensors, (uint8_t *)&msg, sizeof(msg));

		osDelay(50);
		//vTaskSuspend(NULL); // suspend this task
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
