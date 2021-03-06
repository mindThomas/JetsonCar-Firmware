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
 
#ifndef LIGHTANDSOUNDHANDLER_H
#define LIGHTANDSOUNDHANDLER_H

#include "cmsis_os.h"
#include "Priorities.h"

#include "PWM.h"
#include "IO.h"

class LightAndSoundHandler
{
	private:
		const int THREAD_STACK_SIZE = 256; // depending on the resource usage in the main loop (Thread function) you might want to increase this STACK_SIZE value
		const uint32_t THREAD_PRIORITY = APPLICATION_TEMPLATE_PRIORITY; // modify the task priorities in "Priorities.h"

	public:
		LightAndSoundHandler(PWM& red_, PWM& green_, PWM& blue_, IO& buzzer_);
		~LightAndSoundHandler();

		int Start();
		int Stop(uint32_t timeout = 1000);
		int Restart(uint32_t timeout = 1000);

	private:
		void jetColor(const float v, const float vmin, const float vmax, float RGB[3]);

	private:
		static void Thread(void * pvParameters);

	private:
		TaskHandle_t _TaskHandle;
		bool _isRunning;
		bool _shouldStop;

	private:
		PWM& red;
		PWM& green;
		PWM& blue;
		IO& buzzer;

};
	
	
#endif
