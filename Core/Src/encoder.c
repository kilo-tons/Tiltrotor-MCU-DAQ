/*
 * Tiltrotor Propulsion Analyzer DAQ-STM32
 * Copyright (C) 2021 Ho Chi Minh University of Technology, Department of Aerospace Engineering
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "encoder.h"
#include "string.h"

osMutexId Encoder_MutexHandle;
encoder_rpm_data_t encoder_rpm_data;

void encoder_init(encoder_rpm_data_t *input)
{
	memset((void*)input, 0, sizeof(encoder_rpm_data_t));

	//Hard-coded frequency value
	input->freq = 4;
	input->period = 1000 / input->freq;
	input->lin_count = 30;

	// Create thread mutex
	osMutexDef(Encoder_Mutex);
	Encoder_MutexHandle = osMutexCreate(osMutex(Encoder_Mutex));
}

void encoder_read_rpm(encoder_rpm_data_t *input)
{
	uint16_t tick 		= input->tick;
	uint16_t freq 		= input->freq;
	uint16_t lin_count	= input->lin_count;

	// Lock mutex
	osMutexWait(Encoder_MutexHandle, osWaitForever);

	input->rpm = 60*freq*tick / lin_count;
	input->timestamp = HAL_GetTick();

	//Release
	osMutexRelease(Encoder_MutexHandle);

	//Reset tick
	input->tick = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ENCODER_IN_Pin) {
		encoder_rpm_data.tick++;
	}
}
